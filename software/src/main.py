"""
FALCON main entry point.

Startup sequence:
1. Glove (ESP32) must be powered on first to create its WiFi AP
   "FALCON-Glove".
2. Base station connects to:
   - Tello drone on one WiFi interface
   - ESP32 glove AP on a second WiFi interface (via setup_wifi.sh)
3. The glove client thread connects to the glove's TCP server at
   192.168.4.1:5000 and continuously writes finger/IMU data into the
   shared `variables.instruction` dict.
4. The drone thread waits for the first glove packet, takes off, and
   runs the tracking + gesture loop.
5. The main thread owns the OpenCV window. Frames are routed from the
   drone thread through `frame_display_queue` and rendered here.
   (cv2.imshow / cv2.waitKey from a non-main thread is unsupported on
   Linux and was the cause of the window freezing during arc maneuvers.)
"""

import argparse
import queue
import socket
import sys
import threading
import time

import cv2

# NB: the original main.py had `from hardware.firmware import glove_client`
# but the package on disk is `firmware.firmware`. That import error was
# almost certainly why "the glove isn't sending commands" — main.py never
# even reached the line that opens the socket.
from hardware.firmware import glove_client
from software.lib.falcon import FALCON
from software.lib import variables


# Bounded queue. maxsize=2 keeps latency low: if the main thread falls
# behind, the producer drops the oldest frame instead of letting a backlog
# accumulate.
frame_display_queue: "queue.Queue" = queue.Queue(maxsize=2)


def glove_thread_fn(stop_event: threading.Event):
    """
    Glove client thread.

    Connects to the glove's TCP server (with cancellable retry), then runs
    the receive loop until stop_event is set.
    """
    print('[MAIN] Starting glove client thread...')
    sock = glove_client.glove_client_init(stop_event=stop_event)
    if sock is None:
        print('[MAIN] Glove connect cancelled before completion')
        return
    glove_client.run_glove_client(sock, stop_event=stop_event)
    print('[MAIN] Glove thread exiting')


def drone_thread_fn(tello: FALCON, stop_event: threading.Event):
    """
    Drone control thread.

    Waits for the first packet from the glove (via Event, not a busy
    spin on a global bool), then runs the tracking loop. The original
    code did `while not variables.glove_connected: continue`, which
    pegged a CPU core and starved the detector thread.
    """
    print('[MAIN] Starting drone thread...')

    # Wait up to 30s for the glove to come online. If it never does, we
    # still proceed — tracking can run without the glove, just without
    # gesture controls. That's strictly more useful than hanging.
    print('[MAIN] Drone thread waiting for first glove packet...')
    got_glove = variables.glove_ready_event.wait(timeout=30.0)
    if not got_glove and not stop_event.is_set():
        print('[MAIN] Glove never came online — proceeding without gesture control')

    if stop_event.is_set():
        return

    try:
        tello.track_target(frame_display_queue=frame_display_queue)
    except Exception as e:
        print(f'[MAIN] Drone thread crashed: {e}')
        import traceback
        traceback.print_exc()
    finally:
        # Make sure shutdown is signaled even on crash, so main loop exits.
        variables.request_shutdown()
        print('[MAIN] Drone thread exiting')


def main(argv=None):
    parser = argparse.ArgumentParser(description='FALCON drone controller')
    parser.add_argument('--no-glove', action='store_true',
                        help='Skip glove client (drone tracking only)')
    args = parser.parse_args(argv)

    # ---- Initialize Tello ----
    # Done on the main thread because djitellopy spawns its own background
    # threads during construction; we want those rooted in main, not in a
    # daemon thread that might die unexpectedly.
    print('[MAIN] Initializing Tello drone...')
    tello = FALCON()
    print('[MAIN] Tello ready')

    # ---- Window setup ----
    # Window is created on the main thread, which is also where every
    # imshow/waitKey will happen. Keep this invariant.
    window_name = "Tello ArUco Tracker"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 960, 720)
    print(f'[MAIN] Window created: {window_name}')

    # The shutdown event is the single source of truth for "we're stopping".
    # It can be set from: 'q' keypress, KeyboardInterrupt, drone-thread
    # crash, or the 'quit' gesture from inside FALCON. Every loop polls it.
    stop_event = variables.shutdown_event

    # ---- Threads ----
    threads = []

    if not args.no_glove:
        glove_thrd = threading.Thread(
            target=glove_thread_fn, args=(stop_event,),
            name='glove', daemon=False,  # NOT daemon — see comment below
        )
        glove_thrd.start()
        threads.append(glove_thrd)

    drone_thrd = threading.Thread(
        target=drone_thread_fn, args=(tello, stop_event),
        name='drone', daemon=False,
    )
    drone_thrd.start()
    threads.append(drone_thrd)

    # NB: threads are NOT daemons. The original code used daemon=True, which
    # meant if the user pressed 'q' and main returned, Python would kill the
    # threads mid-flight WITHOUT running their finally blocks — meaning
    # tello.land() and tello.streamoff() would never execute. The drone
    # would be left flying. Non-daemon + an explicit shutdown event +
    # joining lets the threads actually clean up.

    print("[MAIN] Press 'q' in the window to land and exit.")

    # ---- Main display loop ----
    try:
        while True:
            # Exit if anyone signaled shutdown
            if stop_event.is_set():
                break

            # Exit if the drone thread is gone — without it there's nothing
            # to display and we should bring everything down.
            if not drone_thrd.is_alive():
                print('[MAIN] Drone thread is no longer alive')
                break

            try:
                frame = frame_display_queue.get(timeout=0.05)
                cv2.imshow(window_name, frame)
            except queue.Empty:
                pass

            # waitKey must be called every iteration on the main thread to
            # pump X11 events, even if no new frame arrived.
            key = cv2.waitKey(10) & 0xFF
            if key == ord('q'):
                print('[MAIN] Exit requested via keyboard')
                stop_event.set()
                break

    except KeyboardInterrupt:
        print('\n[MAIN] Ctrl-C — shutting down')
        stop_event.set()

    finally:
        # Make sure shutdown is signaled, then wait for threads to wind down.
        stop_event.set()

        # Drone thread first — it's the one that needs to land the drone,
        # and it can take a few seconds. Generous timeout.
        print('[MAIN] Waiting for drone thread to land and clean up...')
        drone_thrd.join(timeout=15.0)
        if drone_thrd.is_alive():
            print('[MAIN] WARNING: drone thread did not exit cleanly')

        # Glove thread should exit fast once stop_event is set (it's
        # blocked at most RECV_TIMEOUT seconds in recv).
        if not args.no_glove:
            print('[MAIN] Waiting for glove thread...')
            glove_thrd.join(timeout=5.0)
            if glove_thrd.is_alive():
                print('[MAIN] WARNING: glove thread did not exit cleanly')

        # Drain any leftover frames so destroyAllWindows isn't racing
        # against an in-flight imshow.
        try:
            while not frame_display_queue.empty():
                frame_display_queue.get_nowait()
        except Exception:
            pass

        cv2.destroyAllWindows()
        # Pump events one more time so X11 actually closes the window
        cv2.waitKey(1)
        print('[MAIN] Bye')


if __name__ == '__main__':
    main()
