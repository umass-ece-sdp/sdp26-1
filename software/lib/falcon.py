import subprocess
from pathlib import Path
from djitellopy import Tello
import time
import numpy as np
import cv2
import cv2.aruco as aruco
from queue import Queue, Empty
from threading import Thread
from dataclasses import dataclass
import imageio as iio

@dataclass
class FlightControl:
    YAW_SPEED: int=50
    YAW_DEAD_ZONE: int=25

    TARGET_DIST: float=1.524
    DIST_DEAD_ZONE: float=0.08
    FB_SPEED: int=60
    FB_MAX: int=80

    LR_SPEED: int=50
    LR_DEAD_ZONE: int=20
    LR_MAX: int=60

    VERT_SPEED: int=55
    VERT_DEAD_ZONE: int=20
    VERT_MAX: int=70

    MANUAL_UD_SPEED: int=50

    SMOOTH_WINDOW: int=3

    LOST_TIMEOUT: float=1.0

@dataclass
class FiducialData:
    # --- Config ---
    MARKER_SIZE: float=0.105
    ARUCO_DICT = aruco.DICT_4X4_50
    TARGET_ID: int=3
    VALID_TARGETS: list[int]=[0, 1, 2, 3]

    # --- ArUco Setup ---
    aruco_dict = aruco.getPredefinedDictionary(ARUCO_DICT)
    detector = aruco.ArucoDetector(aruco_dict, aruco.DetectorParameters())

    half = MARKER_SIZE / 2
    obj_points = np.array([
        [-half,  half, 0],
        [ half,  half, 0],
        [ half, -half, 0],
        [-half, -half, 0],
    ], dtype=np.float32)

    # --- Camera calibration ---
    camera_matrix = np.array([
        [921.170702, 0.000000, 459.904354],
        [0.000000, 919.018377, 351.238301],
        [0.000000, 0.000000, 1.000000]
    ], dtype=np.float64)
    dist_coeffs = np.array([-0.033458, 0.105152, 0.001256, -0.006647, 0.000000], dtype=np.float64)


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
        self.finger_thresholds = (0.31, 0.31, 0.31, 0.31)  # V

        # Drone movement commands
        self.move_dist = move_dist  # cm

        # Flight control and fiducial data
        self.flight_control = FlightControl()
        self.aruco_data = FiducialData()

        # Video saving variables
        self.raw_queue = Queue(maxsize=10)
        self.proc_queue = Queue(maxsize=10)
        self.annotated_queue = Queue(maxsize=10)
        self.fourcc = cv2.VideoWriter.fourcc(*'MJPG')
        # self.fourcc = cv2.VideoWriter.fourcc(*'mp4v')
        self.stay_active = False

        # Connect to WiFi before initializing Tello
        # self._connect_wifi()
        
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

    # Currently unused
    def map_fingers(self, fingers: tuple[float, float, float, float]) -> str | None:
        """Maps finger sensors to different drone commands."""
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

    def _video_save_worker(self):
        """Worker thread for saving drone video footage to file."""
        video_writer = None
        while self.stay_active:
            try:
                # Use a timeout so the thread can exit cleanly if no frame is received
                frame = self.raw_queue.get(timeout=0.5)
                # Initialize writer lazily so we know the frame size
                if video_writer is None:
                    h, w = frame.shape[:2]
                    video_writer = cv2.VideoWriter('flight.avi', self.fourcc, 30.0, (1280, 720))
                
                # Resize if necessary and write
                if frame.shape[:2] != (720, 1280):
                    frame_to_write = cv2.resize(frame, (1280, 720))
                else:
                    frame_to_write = frame

                video_writer.write(frame_to_write)
            except Empty:
                continue
            except Exception as e:
                print(f"Video writer error: {e}")

        if video_writer is not None:
            video_writer.release()

    def _target_track_worker(self):
        """Worker thread to process fiducials, update drone speed, and apply overlays."""
        def clamp(val: int, lo: int, hi: int):
            return max(lo, min(hi, val))
            
        dist_history = []
        lr_history = []
        vert_history = []
        last_target_time = time.time()
        
        yaw_cmd = 0
        fb_cmd = 0
        lr_cmd = 0
        ud_cmd = 0
        
        while self.stay_active:
            try:
                frame = self.proc_queue.get(timeout=0.5)
            except Empty:
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = self.aruco_data.detector.detectMarkers(gray)

            frame_h, frame_w = frame.shape[:2]
            center_x = frame_w // 2
            center_y = frame_h // 2
            current_dist = None
            target_found = False

            if ids is not None:
                aruco.drawDetectedMarkers(frame, corners, ids)

                for corner, marker_id in zip(corners, ids.flatten()):
                    success, rvec, tvec = cv2.solvePnP(
                        self.aruco_data.obj_points,
                        corner.reshape(4, 2),
                        self.aruco_data.camera_matrix,
                        self.aruco_data.dist_coeffs,
                    )
                    if not success:
                        continue

                    x, y, z = tvec[0, 0], tvec[1, 0], tvec[2, 0]
                    cv2.drawFrameAxes(
                        frame,
                        self.aruco_data.camera_matrix,
                        self.aruco_data.dist_coeffs,
                        rvec,
                        tvec,
                        0.05
                    )

                    label = f"ID {marker_id}: x={x:.2f} y={y:.2f} z={z:.2f}m"
                    pos = tuple(corner[0][0].astype(int))
                    cv2.putText(frame, label, (pos[0], pos[1] - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    if marker_id == self.aruco_data.TARGET_ID:
                        target_found = True
                        last_target_time = time.time()

                        marker_cx = int(corner[0, :, 0].mean())
                        marker_cy = int(corner[0, :, 1].mean())

                        cv2.circle(frame, (marker_cx, marker_cy), 8, (0, 0, 255), 2)
                        cv2.line(frame, (center_x - 15, center_y),
                                (center_x + 15, center_y), (255, 0, 0), 1)
                        cv2.line(frame, (center_x, center_y - 15),
                                (center_x, center_y + 15), (255, 0, 0), 1)

                        error_x = marker_cx - center_x
                        if abs(error_x) > self.flight_control.YAW_DEAD_ZONE:
                            yaw_cmd = int(clamp(
                                self.flight_control.YAW_SPEED * (error_x / (frame_w / 2)),
                                -100, 100
                            ))
                        else:
                            yaw_cmd = 0

                        raw_dist = np.sqrt(x**2 + y**2 + z**2)
                        dist_history.append(raw_dist)
                        if len(dist_history) > self.flight_control.SMOOTH_WINDOW:
                            dist_history.pop(0)
                        current_dist = np.median(dist_history)

                        dist_error = current_dist - self.flight_control.TARGET_DIST
                        if abs(dist_error) > self.flight_control.DIST_DEAD_ZONE:
                            fb_cmd = int(clamp(
                                self.flight_control.FB_SPEED * (dist_error / self.flight_control.TARGET_DIST),
                                -self.flight_control.FB_MAX,
                                self.flight_control.FB_MAX,
                            ))
                        else:
                            fb_cmd = 0

                        lr_history.append(x)
                        if len(lr_history) > self.flight_control.SMOOTH_WINDOW:
                            lr_history.pop(0)
                        smooth_x = np.median(lr_history)

                        lr_error = smooth_x * (self.aruco_data.camera_matrix[0, 0] / current_dist) if current_dist else 0
                        if abs(lr_error) > self.flight_control.LR_DEAD_ZONE:
                            lr_cmd = int(clamp(
                                self.flight_control.LR_SPEED * (lr_error / (frame_w / 2)),
                                -self.flight_control.LR_MAX,
                                self.flight_control.LR_MAX,
                            ))
                        else:
                            lr_cmd = 0

                        error_y = marker_cy - center_y
                        vert_history.append(error_y)
                        if len(vert_history) > self.flight_control.SMOOTH_WINDOW:
                            vert_history.pop(0)
                        smooth_vert = np.median(vert_history)

                        if abs(smooth_vert) > self.flight_control.VERT_DEAD_ZONE:
                            ud_cmd = int(clamp(
                                -self.flight_control.VERT_SPEED * (smooth_vert / (frame_h / 2)),
                                -self.flight_control.VERT_MAX,
                                self.flight_control.VERT_MAX
                            ))
                        else:
                            ud_cmd = 0

            if not target_found:
                if (time.time() - last_target_time) > self.flight_control.LOST_TIMEOUT:
                    yaw_cmd = 0
                    fb_cmd = 0
                    lr_cmd = 0
                    # Preserve manual ud_cmd set by keyboard by not overriding it or reset it, 
                    # but since this worker doesn't have access to the keyboard event easily, 
                    # we should send 0s for tracking movement. We'll send it all to be safe.
                # ud command input processing would be normally decoupled, but we'll manage here since it overrides
            
            # Draw UI Overlays
            dist_str = f"{current_dist:.2f}m" if current_dist else "N/A"
            line1 = f"TARGET:{self.aruco_data.TARGET_ID}  YAW:{yaw_cmd:+d} FB:{fb_cmd:+d} LR:{lr_cmd:+d} UD:{ud_cmd:+d}"
            line2 = f"DIST:{dist_str} TARGET:{self.flight_control.TARGET_DIST:.2f}m"
            lost_str = "" if target_found else " [TARGET LOST]"
            cv2.putText(frame, line1 + lost_str, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
            cv2.putText(frame, line2, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 2)
            cv2.putText(frame, "0-3=Target  F=Flip  Q=Quit", (10, frame_h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # Send movement command (manual UD commands are trickier across threads so we keep it simple for now or process them in main thread)
            self.send_rc_control(lr_cmd, fb_cmd, ud_cmd, yaw_cmd)

            # Pass the annotated frame to main thread
            if self.annotated_queue.full():
                try: self.annotated_queue.get_nowait()
                except Empty: pass
            self.annotated_queue.put(frame)

    # TODO: Add ability to switch fiducial targeting when moving left/right
        # - left/right commands can be associated to switching the fiducial view
    def track_target(self):
        def convert_video():
            reader = iio.v3.imiter('flight.avi')
            writer = iio.get_writer('flight.mp4', fps=30, codec='libx264')
            for frame in reader:
                writer.append_data(frame)
            writer.close()

        self.streamon()
        time.sleep(2)
        frame_reader = self.get_frame_read()

        print("Taking off...")
        self.takeoff()
        time.sleep(3)

        # Flag and setup for threading
        self.stay_active = True
        video_thread = Thread(target=self._video_save_worker, daemon=True)
        track_thread = Thread(target=self._target_track_worker, daemon=True)
        video_thread.start()
        track_thread.start()

        try:
            while self.stay_active:
                # Capture frame
                frame = frame_reader.frame
                if frame is None:
                    continue

                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

                # Feed queues
                if not self.raw_queue.full():
                    self.raw_queue.put(frame.copy())
                
                # We drop frames on proc_queue if full to prioritize processing the newest frame
                if self.proc_queue.full():
                    try: self.proc_queue.get_nowait()
                    except Empty: pass
                self.proc_queue.put(frame.copy())

                # Display Annotated Frame
                try:
                    annotated_frame = self.annotated_queue.get(timeout=0.01)
                    cv2.imshow("Tello ArUco Tracker", annotated_frame)
                except Empty:
                    # If we don't have an annotated frame yet, we just wait
                    pass

                # --- Keyboard input ---
                key = cv2.waitKey(1) & 0xFF

                if key == ord('q'):
                    self.stay_active = False
                    break
                elif key == ord('f'):
                    self.send_rc_control(0, 0, 0, 0)
                    time.sleep(0.3)
                    try:
                        self.flip("f")
                        time.sleep(1.5)
                    except Exception as e:
                        print(f"Flip failed: {e}")
                    continue
                elif key in [ord('0'), ord('1'), ord('2'), ord('3')]:
                    new_target = key - ord('0')
                    if new_target != self.aruco_data.TARGET_ID:
                        self.aruco_data.TARGET_ID = new_target
                        print(f"Switched to target ID: {self.aruco_data.TARGET_ID}")

        except KeyboardInterrupt:
            print('KeyboardInterrupt detected, shutting down.')
            self.stay_active = False

        finally:
            print("Landing...")
            self.stay_active = False # Ensure threads know to stop
            
            # Briefly wait for threads to clean up
            video_thread.join(timeout=2.0)
            track_thread.join(timeout=2.0)
            
            self.send_rc_control(0, 0, 0, 0)
            time.sleep(0.5)
            self.land()
            self.streamoff()
            cv2.destroyAllWindows()

            # Convert .avi video to .mp4
            convert_video()

if __name__ == '__main__':
    tello = FALCON()
    tello.track_target()