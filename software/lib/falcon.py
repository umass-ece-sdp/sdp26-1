import subprocess
from pathlib import Path
from multiprocessing.connection import Connection
from djitellopy import Tello
from software.lib.fiducials import Fiducial
from software.lib.kalmanfilter import EKF

class FALCON(Tello):
    '''
    The Filming Autonomous Learning and Cinematic Optic Navigator class
    is an extension of DJI's Tello class, using the sdk's functions for
    main drone control in combination with custom functions and
    attributes needed for this project.

    Parameters:
        interface (str, default='wlx90de80899a92'): Interface that the drone
            connects to on the base station.
        ssid (str, default='TELLO-AA7B55'): Tello's WiFi name.
        password (str, default=''): Tello's WiFi password.
        move_dist (int, default=35): Distance that the Tello will move
            when instructed to by the user.
    '''
    def __init__(self, *, interface: str='wlx90de80899a92', ssid: str='TELLO-AA7B55', password: str='', move_dist: int=35):
        self.file_path = Path(__file__).parent
        self.interface = interface
        self.ssid = ssid
        self.password = password

        # Thresholds for determining finger on
        self.finger_thresholds = (2.6, 2.6, 2.6, 2.6)  # V

        # Drone movement commands
        self.move_dist = move_dist  # cm

        # Create the EKF and Fiducial objects
        self.ekf = EKF()
        self.fiducial = Fiducial()

        # Starting target fiducial
        self.target_id = 0 # Chest marker

        # Connect to WiFi before initializing Tello
        self._connect_wifi()
        
        # Initialize normal Tello behavior
        super().__init__()
        
        # Connect to the drone and set it to SDK mode
        self.connect()

        # # Let the environment know the drone is connected
        # variables.set_drone_on()

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

    def map_fingers(self, fingers: tuple[float, float, float, float]) -> str | None:
        # Check finger activations
        f_act = [True if f > thresh else False for f, thresh in zip(fingers, self.finger_thresholds)]

        if f_act[0] and f_act[1] and f_act[2] and f_act[3]:                       # Fist (all active)
            return 'closer'
        elif not (f_act[0] and f_act[1] and f_act[2] and f_act[3]):     # Open hand (all inactive)
            return 'farther'
        elif not (f_act[0]) and f_act[1] and f_act[2] and f_act[3]:                 # Index finger up
            return 'left'
        elif f_act[0] and f_act[1] and f_act[2] and not (f_act[3]):                 # Pinky up
            return 'right'
        elif not (f_act[0] and f_act[1]) and f_act[2] and f_act[3]:             # Peace sign
            return 'land'
        elif f_act[0] and not (f_act[1] and f_act[2]) and f_act[3]:             # Rock & roll
            return 'takeoff'

        return None


    # TODO: Add ability to switch fiducial targeting when moving left/right
        # - left/right commands can be associated to switching the fiducial view
    def track_target(self, pipe_recv: Connection):
        frame_reader = self.get_frame_read()

        try:
            while True:
                frame = frame_reader.frame
                if frame is None:
                    continue

                instructions = pipe_recv.recv()

                # Autonomous controls
                self.ekf.predict_imu(instructions['imu'], instructions['gyro'])
                z_cam = self.fiducial.detect_marker(frame, self.target_id)
                if z_cam is not None:
                    self.ekf.update_camera(z_cam)
                self.ekf.update_uwb(instructions['dist'])
                autonomous_commands = self.ekf.filter_output()
                self.send_rc_control(
                    autonomous_commands['left_right_velocity'],
                    autonomous_commands['forward_backward_velocity'],
                    autonomous_commands['up_down_velocity'],
                    autonomous_commands['yaw_velocity']
                )
                self.set_speed(autonomous_commands['speed'])

                # User input
                command = self.map_fingers(instructions['fingers'])
                match command:  # CW increases fiducial ID, CCW decreases
                    case 'closer':
                        self.move_forward(self.move_dist)
                    case 'farther':
                        self.move_back(self.move_dist)
                    case 'left':    # Move to the side and change fiducial
                        self.move_left(self.move_dist)
                        self.target_id = (self.target_id + 1) % 4 # make sure id doesn't hit >= 4
                    case 'right':   # Move to the side and change fiducial
                        self.move_right(self.move_dist)
                        id_adj = self.target_id - 1
                        self.target_id = id_adj if id_adj > 0 else 3
                    case 'land':
                        self.land()
                    case 'takeoff':
                        self.takeoff()
                    case _: # No commands == skip
                        pass

        except (KeyboardInterrupt, EOFError):
            print('Shutting down drone controller.')

        finally:
            self.land()

if __name__ == '__main__':
    tello = FALCON()