from hardware.firmware import server
import threading, socket
from software.lib.falcon import FALCON
from software.lib import variables

def server_thread(conn: socket.socket, sock: socket.socket):
    '''Thread function to run the server'''
    print('Starting server thread...')
    
    server.run_server(conn, sock)

def drone_thread(tello: FALCON):
    '''Thread function to run the drone controller'''
    print('Starting drone thread...')

    # Wait for glove to be connected before starting drone
    while not variables.glove_connected:
        continue

    tello.track_target()

def main():
    '''Main entry point for the application'''

    # Connect drone to wifi and start AP mode interface for server
    tello = FALCON(
        ssid='TELLO-FE046A',
        password=''
    )

    # Start the server and connect to glove before starting threads
    conn, sock = server.server_init()
    
    # Create the server and drone threads
    server_thrd = threading.Thread(target=server_thread, args=(conn, sock,), daemon=True)
    drone_thrd = threading.Thread(target=drone_thread, args=(tello,), daemon=True)
    
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
    finally:
        # Reset WiFi interfaces
        tello._reset_wifi()

if __name__ == '__main__':
    main()
