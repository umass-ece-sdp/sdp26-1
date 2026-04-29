import socket
import struct
import threading
import time
from typing import Optional

from software.lib import variables

# Connect to the glove's TCP server at 192.168.4.1:5000
GLOVE_HOST = '192.168.4.1'
GLOVE_PORT = 5000
RECONNECT_DELAY = 2  # seconds
PACKET_SIZE = 24     # 6 floats: fingers[4], speed, distance
RECV_TIMEOUT = 3.0   # seconds — how long we wait for a full packet before giving up


def _recv_exact(sock: socket.socket, n: int) -> Optional[bytes]:
    """
    Read exactly `n` bytes from the socket, looping over short reads.

    TCP doesn't preserve message boundaries — a single sock.recv(24) can
    return anywhere from 1 to 24 bytes depending on how the kernel chose
    to deliver the data. The original code treated any short read as a
    closed connection, which caused us to spuriously reconnect mid-stream.

    Returns the full payload, or None if the peer closed.
    """
    buf = bytearray()
    while len(buf) < n:
        try:
            chunk = sock.recv(n - len(buf))
        except socket.timeout:
            return None
        if not chunk:
            return None
        buf.extend(chunk)
    return bytes(buf)


def glove_client_init(stop_event: Optional[threading.Event] = None,
                      connect_timeout: float = 5.0) -> Optional[socket.socket]:
    """
    Connect to the glove's TCP server on the ESP32 AP.

    Parameters:
        stop_event: optional threading.Event — if set, abort retry loop and
            return None. Lets the launcher cancel a hung connect.
        connect_timeout: per-attempt connection timeout in seconds.

    Returns:
        Connected socket, or None if stop_event was set during retry.
    """
    while stop_event is None or not stop_event.is_set():
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(connect_timeout)
        try:
            print(f'[CLIENT] Connecting to glove server at {GLOVE_HOST}:{GLOVE_PORT}...')
            sock.connect((GLOVE_HOST, GLOVE_PORT))
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.settimeout(RECV_TIMEOUT)
            print('[CLIENT] Connected to glove server!')
            variables.set_glove_on()
            return sock
        except (socket.error, OSError) as e:
            print(f'[CLIENT] Connection failed: {e}')
            try:
                sock.close()
            except Exception:
                pass
            print(f'[CLIENT] Retrying in {RECONNECT_DELAY} seconds...')
            slept = 0.0
            while slept < RECONNECT_DELAY:
                if stop_event is not None and stop_event.is_set():
                    return None
                time.sleep(0.1)
                slept += 0.1
    return None


def receive_glove_data(sock: socket.socket) -> Optional[tuple]:
    """
    Receive one sensor packet from glove (24 bytes, 6 little-endian floats).
    Format: fingers[4], speed, distance.
    """
    try:
        packet = _recv_exact(sock, PACKET_SIZE)
        if packet is None:
            return None

        # Single-byte ACK. The firmware now treats ACK as best-effort
        # (no 5-second blocking wait), so a short token is fine.
        try:
            sock.sendall(b'A')
        except (socket.error, OSError):
            return None

        # ESP32 is little-endian; '<ffffff' makes that explicit.
        data = struct.unpack('<ffffff', packet)
        fingers = data[0:4]
        speed = data[4]
        dist = data[5]
        return fingers, speed, dist

    except Exception as e:
        print(f'[CLIENT] Error receiving data: {e}')
        return None


def run_glove_client(sock: socket.socket,
                     stop_event: Optional[threading.Event] = None,
                     debug: bool = False):
    """
    Main client loop. Continuously reads sensor packets and writes them
    into the shared `variables` dict. Auto-reconnects on disconnect.
    """
    t0 = time.time() if debug else 0.0

    try:
        while stop_event is None or not stop_event.is_set():
            data = receive_glove_data(sock)

            if data is not None:
                fingers, speed, dist = data
                variables.write_instr((fingers, speed, dist))

                if debug:
                    instr = variables.read_instr()
                    t = time.time()
                    fingers_v = instr.get('fingers', (0.0, 0.0, 0.0, 0.0))
                    print(
                        '----- Glove Data -----',
                        f"\tFinger sensors: {fingers_v[0]:.3f}, {fingers_v[1]:.3f}, "
                        f"{fingers_v[2]:.3f}, {fingers_v[3]:.3f} V",
                        f"\tSpeed: {instr.get('speed', 0.0):.3f} m/s",
                        f"\tDistance: {instr.get('dist', 0.0):.3f} m",
                        f'\tTime between packets: {t - t0:.3f} s',
                        sep='\n',
                    )
                    t0 = t
            else:
                print('[CLIENT] Lost connection to glove, attempting to reconnect...')
                variables.set_glove_off()
                try:
                    sock.close()
                except Exception:
                    pass
                new_sock = glove_client_init(stop_event=stop_event)
                if new_sock is None:
                    break
                sock = new_sock
                continue

    except KeyboardInterrupt:
        print('\n[CLIENT] Client shutting down...')

    finally:
        try:
            sock.close()
        except Exception:
            pass
        variables.set_glove_off()


if __name__ == '__main__':
    s = glove_client_init()
    if s is not None:
        run_glove_client(s, debug=True)
