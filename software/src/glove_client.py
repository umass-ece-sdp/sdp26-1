import socket
import sys
import struct
from typing import Optional
from software.lib import variables
import time

# Connect to the glove's TCP server at 192.168.4.1:5000
GLOVE_HOST = '192.168.4.1'
GLOVE_PORT = 5000
RECONNECT_DELAY = 2  # seconds

def glove_client_init() -> socket.socket:
    '''
    Connect to the glove's TCP server on the ESP32 AP.
    
    Returns:
        socket: Connected socket to the glove server, or None if connection failed
    '''
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    
    while True:
        try:
            print(f'[CLIENT] Connecting to glove server at {GLOVE_HOST}:{GLOVE_PORT}...')
            sock.connect((GLOVE_HOST, GLOVE_PORT))
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            print(f'[CLIENT] Connected to glove server!')
            variables.set_glove_on()
            return sock
        except socket.error as e:
            print(f'[CLIENT] Connection failed: {e}')
            print(f'[CLIENT] Retrying in {RECONNECT_DELAY} seconds...')
            time.sleep(RECONNECT_DELAY)

def receive_glove_data(sock: socket.socket) -> Optional[tuple]:
    """
    Receive sensor packet from glove (24 bytes).
    Format: 6 floats (fingers[4], speed, distance)
    
    Parameters:
        sock (socket): Connected socket to glove server
    
    Returns:
        tuple: (fingers, speed, distance) or None if connection closed
    """
    try:
        # Receive exactly 24 bytes
        packet = sock.recv(24)
        
        if not packet or len(packet) < 24:
            print("[CLIENT] Connection closed by glove")
            return None
        
        # Send ACK back to glove
        sock.send(b'ACK\n')
        
        # Unpack struct: 6 floats
        data = struct.unpack('ffffff', packet)
        fingers = data[0:4]
        speed = data[4]
        dist = data[5]

        return fingers, speed, dist
        
    except Exception as e:
        print(f"[CLIENT] Error receiving data: {e}")
        return None

def run_glove_client(sock: socket.socket, debug: bool = False):
    """
    Main client loop that continuously receives sensor data from the glove.
    """
    
    # Start tracking time if in 'debug' mode
    if debug:
        t0 = time.time()
    
    try:
        while True:
            # Receive glove sensor data
            data = receive_glove_data(sock)

            # Write the data to shared variable
            if data:
                fingers, speed, dist = data
                variables.write_instr((fingers, speed, dist))
            else:
                # Connection lost, break out to reconnect
                print("[CLIENT] Lost connection to glove, attempting to reconnect...")
                variables.set_glove_off()
                sock.close()
                sock = glove_client_init()
                continue
            
            # Print received data if in debug mode
            if debug:
                instr = variables.read_instr()
                t = time.time()
                print(
                    '----- Glove Data -----',
                    f'\tFinger sensors: {instr["fingers"][0]:.3f}, {instr["fingers"][1]:.3f}, {instr["fingers"][2]:.3f}, {instr["fingers"][3]:.3f} V',
                    f'\tSpeed: {instr["speed"]:.3f} m/s',
                    f'\tDistance: {instr["dist"]:.3f} m',
                    f'\tTime between packets: {t - t0:.3f} s',
                    sep='\n',
                )
                t0 = t
            
    except KeyboardInterrupt:
        print("\n[CLIENT] Client shutting down...")
            
    finally:
        sock.close()
        variables.set_glove_off()

if __name__ == '__main__':
    sock = glove_client_init()
    run_glove_client(sock, debug=True)
