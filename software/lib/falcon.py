import socket
import threading
import subprocess
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

    # def send_command_with_return(self, command: str, timeout: int = Tello.RESPONSE_TIMEOUT) -> str:
    #     """Send command to Tello and wait for its response.
    #     Internal method, you normally wouldn't call this yourself.
    #     Return:
    #         bool/str: str with response text on success, False when unsuccessfull.
    #     """
    #     # Commands very consecutive makes the drone not respond to them.
    #     # So wait at least self.TIME_BTW_COMMANDS seconds
    #     diff = time.time() - self.last_received_command_timestamp
    #     if diff < self.TIME_BTW_COMMANDS:
    #         self.LOGGER.debug('Waiting {} seconds to execute command: {}...'.format(diff, command))
    #         time.sleep(diff)

    #     self.LOGGER.info("Send command: '{}'".format(command))
    #     timestamp = time.time()

    #     client_socket.sendto(command.encode('utf-8'), self.address)

    #     responses = self.get_own_udp_object()['responses']

    #     while not responses:
    #         if time.time() - timestamp > timeout:
    #             message = "Aborting command '{}'. Did not receive a response after {} seconds".format(command, timeout)
    #             self.LOGGER.warning(message)
    #             return message
    #         time.sleep(0.1)  # Sleep during send command

    #     self.last_received_command_timestamp = time.time()

    #     first_response = responses.pop(0)  # first datum from socket
    #     try:
    #         response = first_response.decode("utf-8")
    #     except UnicodeDecodeError as e:
    #         self.LOGGER.error(e)
    #         return "response decode error"
    #     response = response.rstrip("\r\n")

    #     self.LOGGER.info("Response {}: '{}'".format(command, response))
    #     return response

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