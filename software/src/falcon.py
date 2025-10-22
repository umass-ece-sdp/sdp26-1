import socket
import threading
import subprocess
import cv2
from functools import partial
from djitellopy import Tello

class FALCON(Tello):
    '''
    The Filming Autonomous Learning and Cinematic Optic Navigator class
    is an extension of DJI's Tello class, using the sdk's functions for
    main drone control in combination with custom functions and
    attributes needed for this project.

    Parameters:
        speed (int, default=10): Initial speed in cm/s that the drone
            will fly at
        distance (int, default=20): Initial distance that the drone
            moves when told to move a direction
        degrees (int, default=10): Initial degrees the drone will turn
    '''
    def __init__(self, speed: int=10, distance: int=20, degrees: int=10):
        super().__init__()

        # Initialize variables
        self.speed = speed
        self.distance = distance
        self.degrees = degrees
        self._init_actions()

        # Video streaming variables
        self.stream_on = False
        self.video_thread = None
        self.frame_reader = None

        # Connect to the drone and set it to SDK mode first
        # TODO: Drone ssid here
        # self._connect_wifi()    # Only works on base station, commment out for testing on laptop
        self.connect()

        # Create and bind a socket to the drone (after connection established)
        # Receive output from drone
        self._open_socket()
        recvThread = threading.Thread(target=self._recv)
        recvThread.daemon = True
        recvThread.start()

        # Set the initial speed
        self.set_speed(self.speed)

    def check_drone_ready(self) -> bool:
        '''
        Checks if drone is ready for operations.
        Returns:
            bool: Equal to `True` if the drone is ready to fly,
            `False` if the battery is too low.
        '''
        print('=== DRONE STATUS CHECK ===')
        status = self._get_drone_status()
        if status:
            print(f'Battery: {status["battery"]}%')
            print(f'Height: {status["height"]}cm') 
            print(f'Temperature: {status["temperature"]}°C')
            
            # Check if ready for takeoff
            if status['battery'] < 20:
                print('⚠️  WARNING: Low battery - charge before flying!')
            elif status['battery'] < 10:
                print('❌ CRITICAL: Battery too low!')
                return False
            else:
                print('✅ Drone appears ready for flight')
        else:
            print('❌ Could not get drone status')
            return False
        print('========================\n')
        return True

    def start_video_stream(self) -> None:
        '''
        Starts the video stream from the drone and displays it in a window.
        Also handles keyboard input for drone control.
        '''
        # Check drone status first
        if not self.check_drone_ready():
            print('Drone not ready - check issues above')
            return
            
        # Start video stream
        self.streamon()
        self.stream_on = True
        
        # Get the video stream
        self.frame_reader = self.get_frame_read()
        
        print('Video stream started. Use keyboard controls:')
        print('\tWASD - Move forward/left/back/right')
        print('\tQE - Rotate left/right')
        print('\tRF - Move up/down')
        print('\tT - Takeoff (with safety checks)')
        print('\tL - Land')
        print('\tESC - Exit')
        
        # Start video display loop
        self._video_loop()

    def safe_takeoff(self) -> None:
        '''
        Performs pre-takeoff checks before attempting takeoff.
        '''
        print('Performing pre-takeoff checks...')
        
        # Check battery level
        try:
            battery = self.get_battery()
            print(f'Battery level: {battery}%')
            if battery < 20:
                print('⚠️  WARNING: Battery level is low! Consider charging before takeoff.')
                return
            if battery < 10:
                print('❌ CRITICAL: Battery too low for takeoff!')
                return
        except Exception as e:
            print(f'Could not check battery: {e}')
        
        # Check if already flying
        try:
            height = self.get_height()
            print(f'Current height: {height}cm')
            if height > 5:  # If already more than 5cm off ground
                print('❌ Drone appears to already be flying!')
                return
        except Exception as e:
            print(f'Could not check height: {e}')
        
        # Check temperature
        try:
            temp = self.get_temperature()
            print(f'Temperature: {temp}°C')
            if temp > 80:
                print('⚠️  WARNING: High temperature detected!')
        except Exception as e:
            print(f'Could not check temperature: {e}')
        
        print('Attempting takeoff...')
        try:
            self.takeoff()
            print('✅ Takeoff successful!')
        except Exception as e:
            print(f'❌ Takeoff failed: {e}')

    def _stop_video_stream(self) -> None:
        '''
        Stops the video stream and closes the display window.
        '''
        if self.stream_on == False:
            print('Stream already off.')
            return
        self.stream_on = False
        if hasattr(self, 'frame_reader') and self.frame_reader:
            self.streamoff()
        cv2.destroyAllWindows()
        print('Video stream stopped.')

    def _get_drone_status(self) -> dict | None:
        '''
        Returns current drone status information.
        '''
        try:
            battery = self.get_battery()
            height = self.get_height()
            temp = self.get_temperature()
            return {
                'battery': battery,
                'height': height,
                'temperature': temp
            }
        except Exception as e:
            print(f'Error getting drone status: {e}')
            return None

    def _video_loop(self) -> None:
        '''
        Main video display and keyboard input loop.
        '''
        while self.stream_on:
            # Get frame from drone
            frame = self.frame_reader.frame
            
            # Convert from BGR to RGB to fix inverted colors
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # Display the frame
            cv2.imshow('Tello Video Stream', frame)
            
            # Handle keyboard input (1ms wait)
            key = cv2.waitKey(1) & 0xFF
            
            if key == 27:  # ESC key
                break
            elif key == 255:  # No key pressed
                continue
            self.actions[key]() if key in self.actions else print('Key not mapped.')
        
        # Cleanup
        self._stop_video_stream()

    def _open_socket(self) -> None:
        '''
        Helper function to initialize and bind a socket to the drone.
        '''
        host = ''
        port = 9000
        self.telloaddr = ('192.168.10.1', 8889)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((host, port))

    def _recv(self) -> None:
        '''
        Continuously listens for incoming UDP packets from the drone
        and prints the contents of the package.
        '''
        while True:
            try:
                data, server = self.sock.recvfrom(1518)
                print(data.decode(encoding='utf-8'))
            except Exception:
                print('\nExit . . .\n')
                break

    def _connect_wifi(self, interface: str='wlan0', ssid: str='', password: str='') -> None:
        '''
        Automatically connects Linux devices to the drone using a bash
        script stored in software/scripts. ***This will only work for
        Linux devices, Windows users will need to conenct manually.***
        '''
        # Call to bash script to connect WiFi
        cmd = [r'../scripts/connection_client.sh', interface, ssid, password]

        # Error checking
        try:
            result = subprocess.run(cmd, check=True, capture_output=True, text=True)
            print(f'STDOUT: {result.stdout}')
            print('Connected successfully.')
        except subprocess.CalledProcessError as e:
            print("Command failed with exit", e.returncode)
            print("STDOUT:\n", e.stdout)
            print("STDERR:\n", e.stderr)

    def _init_actions(self):
        '''
        Actions to relate different key presses to.
        '''
        self.actions = {
            ord('t'): self.safe_takeoff,
            ord('l'): self.land,
            ord('w'): partial(self.move_forward, self.distance),
            ord('a'): partial(self.move_left, self.distance),
            ord('s'): partial(self.move_back, self.distance),
            ord('d'): partial(self.move_right, self.distance),
            ord('r'): partial(self.move_up, self.distance),
            ord('f'): partial(self.move_down, self.distance),
            ord('q'): partial(self.rotate_counter_clockwise, self.degrees),
            ord('e'): partial(self.rotate_clockwise, self.degrees),
        }

if __name__ == '__main__':
    FALCON._connect_wifi(ssid='62A-summer', password='62Asummer!')
    exit()
    
    tello = FALCON()
    tello.start_video_stream()
    tello.end()