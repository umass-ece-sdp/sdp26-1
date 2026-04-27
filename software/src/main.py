from hardware.firmware import glove_client
import threading, socket, queue
import cv2
from software.lib.falcon import FALCON
from software.lib import variables

# Thread-safe queue for passing display frames from drone thread to main thread
frame_display_queue = queue.Queue(maxsize=2)

def glove_thread(glove_sock: socket.socket):
    '''Thread function to receive glove data'''
    print('Starting glove client thread...')
    
    glove_client.run_glove_client(glove_sock)

def drone_thread(tello: FALCON):
    '''Thread function to run the drone controller'''
    print('Starting drone thread...')

    # Wait for glove to be connected before starting drone
    while not variables.glove_connected:
        continue

    tello.track_target(frame_display_queue)

def main():
    '''
    Main entry point for the application.
    
    Startup sequence:
    1. Glove (ESP32) must be powered on first to create its WiFi AP "FALCON-Glove"
    2. Base station connects to:
       - Tello drone on one WiFi interface
       - ESP32 glove AP on second WiFi interface (via setup_wifi.sh)
    3. Base station connects to glove's TCP server at 192.168.4.1:5000
    4. Drone control loop reads glove instructions and commands the Tello
    '''

    # Initialize Tello drone (connects to Tello's WiFi SSID)
    # Note: Base station must already be connected to Tello before this step
    tello = FALCON(
        # ssid='TELLO-FE046A',
        # password=''
    )

    # Connect to the glove's TCP server
    # (Base station must already be connected to ESP32 AP via setup_wifi.sh)
    glove_sock = glove_client.glove_client_init()
    
    # Create OpenCV window on MAIN THREAD before starting drone thread
    # (OpenCV window creation AND updates must happen on the main thread on Linux)
    # NOTE: No longer using cv2.startWindowThread() - display happens in main loop below
    window_name = "Tello ArUco Tracker"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 960, 720)
    print(f"[DEBUG] OpenCV window created on main thread: {window_name}")
    
    # Create the glove and drone threads
    glove_thrd = threading.Thread(target=glove_thread, args=(glove_sock,), daemon=True)
    drone_thrd = threading.Thread(target=drone_thread, args=(tello,), daemon=True)
    
    # Start both threads
    glove_thrd.start()
    drone_thrd.start()
    
    print("[DEBUG] Threads started. Keeping main thread alive for window event processing...")
    
    try:
        # Main thread must display frames AND process window events on Linux
        # Daemon thread puts frames in queue, main thread displays them
        display_count = 0
        while glove_thrd.is_alive() or drone_thrd.is_alive():
            try:
                # Try to get a frame from the queue (non-blocking with 50ms timeout)
                # This lets us process window events frequently
                frame = frame_display_queue.get(timeout=0.050)
                display_count += 1
                
                # CRITICAL: Do cv2.imshow on MAIN THREAD on Linux!
                cv2.imshow(window_name, frame)
                
                if display_count % 30 == 0:
                    print(f"[MAIN] Displayed {display_count} frames")
                    
            except queue.Empty:
                # No frame in queue, that's OK - just process window events
                pass
            
            # Process window events and keypresses
            key = cv2.waitKey(10)  # Short timeout for responsive window
            if key == ord('q'):
                print("[DEBUG] 'q' pressed - exiting")
                break
    except KeyboardInterrupt:
        print('\nShutting down threads...')
    finally:
        cv2.destroyAllWindows()
        print('Threads terminated.')
        # Reset WiFi interfaces (disconnect from Tello)
        # tello._reset_wifi()

if __name__ == '__main__':
    main()
