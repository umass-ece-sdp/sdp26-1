from hardware.firmware import server
import threading, socket
from software.lib import falcon_vision as fv
from software.lib import variables
import subprocess
from pathlib import Path

def server_thread(conn: socket.socket, sock: socket.socket):
    '''Thread function to run the server'''
    print('Starting server thread...')
    
    # # Wait for the drone to be connected before looking for instructions
    # while not variables.drone_connected:
    #     continue
    
    server.run_server(conn, sock)

def drone_thread():
    '''Thread function to run the drone controller'''
    print('Starting drone thread...')

    # Wait for glove to be connected before starting drone
    while not variables.glove_connected:
        continue
    
    fv.run_tracking()

def main():
    '''Main entry point for the application'''

    color = input('Target color: ')
    variables.set_color(color)

    # Configure the AP interface to be the correct IP address
    subprocess.run(['bash', Path(__file__).parent.parent.joinpath('scripts', 'config_ap.sh')], check=True)

    # Start the server and connect to glove before starting threads
    conn, sock = server.server_init()
    
    # Create the server and drone threads
    server_thrd = threading.Thread(target=server_thread, args=(conn, sock,), daemon=True)
    drone_thrd = threading.Thread(target=drone_thread, daemon=True)
    
    # Start both threads
    server_thrd.start()
    drone_thrd.start()
    
    try:
        # Wait for both threads to complete
        server_thrd.join()
        drone_thrd.join()
    except KeyboardInterrupt:
        print('\nShutting down threads...')
        print('Threads terminated.')

if __name__ == '__main__':
    main()