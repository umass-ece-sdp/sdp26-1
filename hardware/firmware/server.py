import socket
import sys
import struct
from multiprocessing.synchronize import Event
from multiprocessing.connection import Connection
from typing import Optional

HOST = '192.168.20.1'
PORT = 5000

def server_init(glove_event: Event) -> tuple[socket.socket, socket.socket]:
    '''
    Create a socket for the glove-controller to bind to.

    Parameters:
        glove_event (Event): Multiprocessing event set when the glove connects.

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
    glove_event.set()
    return conn, sock

def receive_instructions(conn: socket.socket) -> Optional[tuple[tuple, tuple, tuple, tuple]]:
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
        packet = conn.recv(32)
        
        if not packet or len(packet) < 32:
            print("Connection closed or incomplete data")
            return None
        
        # Unpack struct
        data = struct.unpack('fffffffffff', packet)
        fingers = data[0:4]
        imu = data[4:7]
        gyro = data[7:10]
        dist = data[10]
        
        # Send ACK back to client
        conn.send(b'ACK')

        return fingers, imu, gyro, dist
        
    except Exception as e:
        print(f"Error receiving data: {e}")
        return None

def run_server(conn: socket.socket, sock: socket.socket, pipe_send: Connection):
    """
    Main server loop that continuously receives data from the client.
    """
    try:
        print('Waiting for instructions...')
        while True:
            # Receive instruction from ESP32
            instruction = receive_instructions(conn)

            # Send the instruction to the drone process via pipe
            if instruction:
                pipe_send.send(instruction)

            print(instruction)
            
    except KeyboardInterrupt:
        print("\nServer shutting down...")
            
    finally:
        conn.close()
        sock.close()
        pipe_send.close()
        print('Glove disconnected.')

if __name__ == '__main__':
    import multiprocessing
    from multiprocessing import Pipe
    _glove_event = multiprocessing.Event()
    _pipe_recv, _pipe_send = Pipe(duplex=False)
    conn, sock = server_init(_glove_event)
    run_server(conn, sock, _pipe_send)