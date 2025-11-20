import socket
import sys
import struct
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
        print(f'Bind failed. Error code: {str(message[0])} Message {message[1]}')
        sys.exit()

    print('Binding complete')

    # Listen on the port for connections
    sock.listen(9)
    conn, addr = sock.accept()
    print(f'Connected with {addr[0]}: {(str(addr[1]))}')
    return conn, sock

def receive_instructions(conn: socket.socket):
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
        # Receive exactly 4 bytes
        data = conn.recv(4)
        
        if not data or len(data) < 4:
            print("Connection closed or incomplete data")
            return None
        
        # Convert to string (4 characters)
        value = data.decode('utf-8')
        
        # Verify it's exactly 4 characters
        if len(value) != 4:
            print(f"Invalid string length: expected 4, got {len(value)}")
            return None
        
        print(f"Received string: {value}")
        
        # Send ACK back to client
        conn.send(b'ACK')
        
        return value
        
    except Exception as e:
        print(f"Error receiving data: {e}")
        return None

def run_server(shared_dict: dict, conn: socket.socket, sock: socket.socket):
    """
    Main server loop that continuously receives data from the client.
    
    Parameters:
        shared_dict: multiprocessing.Manager dict for sharing variables
    """
    # Initialize shared variables
    variables.init_shared(shared_dict)
    
    try:
        print('Waiting for instructions...')
        while True:
            # Receive instruction from ESP32
            instruction = receive_instructions(conn)

            # Write the instruction to shared dict
            variables.write_instr(instruction)

            print(instruction)
            
    except KeyboardInterrupt:
        print("\nServer shutting down...")
            
    finally:
        conn.close()
        sock.close()

if __name__ == '__main__':
    run_server()