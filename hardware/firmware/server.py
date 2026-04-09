import socket
import sys
import struct
from typing import Optional
from software.lib import variables

HOST = '192.168.20.1'
PORT = 5000

def server_init() -> tuple[socket.socket, socket.socket]:
    '''
    Create a socket for the glove-controller to bind to.

    Returns:
        tuple (socket, socket):
        - Information for client bound to server
        - Information for open server socket
    '''

    # Create a socket and try binding
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.bind((HOST, PORT))
    except socket.error as message:
        print(f'Bind failed. Error: {message}')
        sys.exit()

    print('Binding complete')

    # Listen on the port for connections
    sock.listen(9)
    conn, addr = sock.accept()
    print(f'Connected with {addr[0]}: {(str(addr[1]))}')
    variables.set_glove_on()
    return conn, sock

def receive_instructions(conn: socket.socket) -> Optional[tuple[tuple, tuple, tuple]]:
    """
    Receive a 4-byte string from the ESP32 client.
    The ESP32 sends the string in network byte order (big-endian).
    
    Parameters:
        conn (socket): Socket representing bound client.
    
    Returns:
        str: The received 4-character string, or None if connection closed.
    """
    try:
        print('Reading instruction...')
        # Receive exactly 32 bytes
        packet = conn.recv(24)
        
        if not packet or len(packet) < 24:
            print("Connection closed or incomplete data")
            return None
        
        # Unpack struct
        data = struct.unpack('ffffff', packet)
        fingers = data[0:4]
        speed = data[4]
        dist = data[5]
        
        # Send ACK back to client
        conn.send(b'ACK')

        return fingers, speed, dist
        
    except Exception as e:
        print(f"Error receiving data: {e}")
        return None

def run_server(conn: socket.socket, sock: socket.socket):
    """
    Main server loop that continuously receives data from the client.
    """
    try:
        print('Waiting for instructions...')
        while True:
            # Receive instruction from ESP32
            instruction = receive_instructions(conn)

            # Write the instruction to shared variable
            if instruction:
                variables.write_instr(instruction)

            print(instruction)
            
    except KeyboardInterrupt:
        print("\nServer shutting down...")
            
    finally:
        conn.close()
        sock.close()
        variables.set_glove_off()

if __name__ == '__main__':
    conn, sock = server_init()
    run_server(conn, sock)