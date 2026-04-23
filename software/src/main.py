from hardware.firmware import glove_client
import threading, socket
from software.lib.falcon import FALCON
from software.lib import variables

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

    tello.track_target()

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
        ssid='TELLO-FE046A',
        password=''
    )

    # Connect to the glove's TCP server
    # (Base station must already be connected to ESP32 AP via setup_wifi.sh)
    glove_sock = glove_client.glove_client_init()
    
    # Create the glove and drone threads
    glove_thrd = threading.Thread(target=glove_thread, args=(glove_sock,), daemon=True)
    drone_thrd = threading.Thread(target=drone_thread, args=(tello,), daemon=True)
    
    # Start both threads
    glove_thrd.start()
    drone_thrd.start()
    
    try:
        # Wait for both threads to complete
        glove_thrd.join()
        drone_thrd.join()
    except KeyboardInterrupt:
        print('\nShutting down threads...')
        print('Threads terminated.')
    finally:
        # Reset WiFi interfaces (disconnect from Tello)
        tello._reset_wifi()

if __name__ == '__main__':
    main()


if __name__ == '__main__':
    main()
