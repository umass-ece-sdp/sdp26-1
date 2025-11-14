import socket
import threading
import subprocess
from pathlib import Path
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
    def __init__(self):
        super().__init__()

        # Initialize variables
        # self._init_actions()
        self.file_path = Path(__file__).parent

        # Connect to the drone and set it to SDK mode first
        # TODO: Drone ssid here
        self._connect_wifi()    # Only works on base station, commment out for testing on laptop, add ssid and password
        self.connect()

        # Create and bind a socket to the drone (after connection established)
        # Receive output from drone
        self._open_socket()
        recvThread = threading.Thread(target=self._recv)
        recvThread.daemon = True
        recvThread.start()

    def _open_socket(self) -> None:
        '''
        Helper function to initialize and bind a socket to the drone.
        '''
        host = 'wlP1p1s0'   # Hard code the on-board WiFi card
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

    def _connect_wifi(self, interface: str='wlx90de80899a92', ssid: str='TELLO-AA7B55', password: str='') -> None:
        '''
        Automatically connects Linux devices to the drone using a bash
        script stored in software/scripts. Searches, starting from the
        working directory, until it finds the script. ***This will
        only work for Linux devices, Windows users will need to
        connect manually. Run any scripts containing this function
        from the parent directory of the repository.***
        '''
        # Call to bash script to connect WiFi
        path_to_script = self.file_path.parent.joinpath(r'/scripts/connection_client.sh').as_posix()
        command =  f'source {path_to_script}'
        cmd = [command, interface, ssid, password]

        # Error checking
        try:
            result = subprocess.run(cmd, check=True, capture_output=True, text=True)
            print('STDOUT:\n', result.stdout)
            print('Connected successfully.')
        except subprocess.CalledProcessError as e:
            print("Command failed with exit", e.returncode)
            print('STDOUT:\n', e.stdout)
            print('STDERR:\n', e.stderr)
            print('Cannot connect to Tello\'s WiFi, exiting...')
            exit()

"""
    def _init_actions(self):
        '''
        Actions to relate different key presses to.
        '''
        self.actions = {
            '': self.land,
            '': partial(self.move_forward, self.distance),
            '': partial(self.move_left, self.distance),
            '': partial(self.move_back, self.distance),
            '': partial(self.move_right, self.distance),
            '': partial(self.move_up, self.distance),
            '': partial(self.move_down, self.distance),
            '': partial(self.rotate_counter_clockwise, self.degrees),
            '': partial(self.rotate_clockwise, self.degrees),
        }
"""