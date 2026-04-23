from hardware.firmware import server
import threading, socket, time
from software.lib.falcon import FALCON
from software.lib import variables

def server_thread(conn: socket.socket, sock: socket.socket):
    '''Thread function to run the server'''
    print('Starting server thread...')
    
    server.run_server(conn, sock)

def main():
    '''Main entry point for the application'''

    # Connect drone to wifi and start AP mode interface for server
    tello = FALCON(
        ssid='TELLO-FE046A',
        password=''
    )

    # Give the hotspot time to fully initialize
    print('Waiting for hotspot to initialize...')
    time.sleep(2)
    
    # Start the server and connect to glove before starting threads
    # NOW the hotspot should be ready to bind to 192.168.20.1
    conn, sock = server.server_init()
    
    # Create the server thread (background task)
    server_thrd = threading.Thread(target=server_thread, args=(conn, sock,), daemon=True)
    
    # Start server thread
    server_thrd.start()
    
    # Wait for glove to be connected before starting drone
    print('Waiting for glove connection...')
    while not variables.glove_connected:
        continue
    
    print('Glove connected, starting drone tracking...')
    
    try:
        # Run tracking on main thread so OpenCV display window works properly
        tello.track_target()
    except KeyboardInterrupt:
        print('\nShutting down...')
    finally:
        # Reset WiFi interfaces
        tello._reset_wifi()

if __name__ == '__main__':
    main()
