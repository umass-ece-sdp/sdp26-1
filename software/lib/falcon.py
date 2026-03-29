import subprocess
import platform
from software.lib.wifi_manager import LinuxWifiManager, WindowsWifiManager, MacWifiManager
from pathlib import Path
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
    def __init__(self, ssid: str='TELLO-AA7B55', password: str=''):
        self.file_path = Path(__file__).parent
        self.tello_ssid = ssid
        self.tello_password = password

        # Connect to WiFi before initializing Tello
        # self._connect_wifi()
        
        # Initialize normal Tello behavior
        super().__init__()
        
        # Connect to the drone and set it to SDK mode
        # self.connect()

        # # Let the environment know the drone is connected
        # variables.set_drone_on()
    
    def setup_wifi(self):
        def get_manager():
            system = platform.system().lower()
            match system:
                case 'linux':
                    return LinuxWifiManager()
                case 'windows':
                    return WindowsWifiManager()
                case 'darwin':
                    return MacWifiManager()
                case _:
                    raise RuntimeError('Unsupported OS')

        self.manager = get_manager()
        iface1, iface2 = self.manager.discover_interfaces()
        ap_ssid = 'jetson_nano_wifi'
        ap_password = 'team1-falcon'
        try:
            print(f"Using interfaces: {iface1}, {iface2}")
            self.manager.connect(iface1, self.tello_ssid, self.tello_password)
            self.manager.set_ap_mode(iface2, ap_ssid, ap_password)
        except Exception as error:
            print(error)
    
    def teardown_wifi(self):
        self.manager.restore()

    def _connect_wifi(self) -> None:
        '''
        Automatically connects Linux devices to the drone using a bash
        script stored in software/scripts. Searches, starting from the
        working directory, until it finds the script. ***This will
        only work for Linux devices, Windows users will need to
        connect manually. Run any scripts containing this function
        from the parent directory of the repository.***
        '''
        def get_wifi_interfaces():
            cmd = ['nmcli', '-t', '-f', 'DEVICE,TYPE', 'device']
            result = subprocess.run(cmd, capture_output=True, text=True, check=True)
            wifi_interfaces = []
            for line in result.stdout.strip().split('\n'):
                device, dev_type = line.split(':')
                if dev_type == 'wifi':
                    wifi_interfaces.append(device)
            if len(wifi_interfaces) < 2:
                raise RuntimeError('At least two WiFi interfaces are required.')
            return wifi_interfaces

        # Call to bash script to setup WiFi
        path_to_script = self.file_path.parent.joinpath('scripts', 'setup.sh').as_posix()
        interfaces = get_wifi_interfaces()
        self.ap_interface, self.client_interface = interfaces[0], interfaces[1]
        ap_ssid = 'jetson_nano_wifi'
        ap_password = 'team1-falcon'
        cmd = ['bash', path_to_script, self.ap_interface, ap_ssid, ap_password, self.client_interface, self.tello_ssid, self.tello_password]

        # Error checking
        try:
            result = subprocess.run(cmd, check=True, capture_output=True, text=True)
            print('STDOUT:\n', result.stdout)
            print('Connected successfully.')
        except subprocess.CalledProcessError as e:
            print('Command failed with exit', e.returncode)
            print('STDOUT:\n', e.stdout)
            print('STDERR:\n', e.stderr)
            print('Cannot connect to Tello\'s WiFi, exiting...')
            exit()

    def wifi_cleanup(self):
        path_to_script = self.file_path.parent.joinpath('scripts', 'teardown.sh').as_posix()
        cmd = ['bash', path_to_script, self.ap_interface, self.client_interface]
        subprocess.run(cmd, check=True, capture_output=True, text=True)

if __name__ == '__main__':
    tello = FALCON()
    tello.setup_wifi()
    import time
    time.sleep(5)
    tello.teardown_wifi()