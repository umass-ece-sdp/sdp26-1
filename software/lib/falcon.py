import socket
import threading
import subprocess
import time
from pathlib import Path
from functools import partial
from djitellopy import Tello

# client_socket: socket.socket

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
    def __init__(self, interface: str='wlx90de80899a92', ssid: str='TELLO-AA7B55', password: str=''):
        self.file_path = Path(__file__).parent
        self.interface = interface
        self.ssid = ssid
        self.password = password

        # Connect to WiFi before initializing Tello
        self._connect_wifi()
        
        # Initialize normal Tello behavior
        super().__init__()
        
        # Connect to the drone and set it to SDK mode
        self.connect()

    def _connect_wifi(self) -> None:
        '''
        Automatically connects Linux devices to the drone using a bash
        script stored in software/scripts. Searches, starting from the
        working directory, until it finds the script. ***This will
        only work for Linux devices, Windows users will need to
        connect manually. Run any scripts containing this function
        from the parent directory of the repository.***
        '''
        # Call to bash script to connect WiFi
        path_to_script = self.file_path.parent.joinpath('scripts', 'connection_client.sh').as_posix()
        cmd = ['bash', path_to_script, self.interface, self.ssid, self.password]

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

if __name__ == '__main__':
    tello = FALCON()