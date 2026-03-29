import subprocess
from pathlib import Path
from djitellopy import Tello
import time
import numpy as np
import cv2
import cv2.aruco as aruco
from software.lib import variables
from dataclasses import dataclass, field

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

    # --- Orbit config ---
    ORBIT_YAW_SPEED = 35        # Yaw rate during orbit (deg/s-ish, RC units)
    ORBIT_LATERAL_SPEED = 30    # Lateral strafe to maintain arc
    ORBIT_CHECK_INTERVAL = 0.5  # Seconds between marker checks during orbit
    ORBIT_TIMEOUT = 30.0        # Max seconds to orbit before giving up

@dataclass
class FiducialData:
    # --- Config ---
    MARKER_SIZE: float=0.105
    ARUCO_DICT = aruco.DICT_4X4_50
    TARGET_ID: int=0
    VALID_TARGETS: list[int]=field(default_factory=lambda: [0, 1, 2, 3])

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

    # TODO: Add ability to switch fiducial targeting when moving left/right
        # - left/right commands can be associated to switching the fiducial view
    def track_target(self):
        def clamp(val: int, lo: int, hi: int):
            return max(lo, min(hi, val))
        
        def clear_histories():
            """Reset smoothing buffers when switching targets."""
            dist_history.clear()
            lr_history.clear()
            vert_history.clear()
        
        def status_display():
            """Displays frame and ArUco tracking information."""
            if frame is None:
                return
            dist_str = f"{current_dist:.2f}m" if current_dist else "N/A"
            line1 = f"TARGET:{self.aruco_data.TARGET_ID}  YAW:{yaw_cmd:+d} FB:{fb_cmd:+d} LR:{lr_cmd:+d} UD:{ud_cmd:+d}"
            line2 = f"DIST:{dist_str} TARGET:{self.flight_control.TARGET_DIST:.2f}m"
            lost_str = "" if target_found else " [TARGET LOST]"
            cv2.putText(frame, line1 + lost_str, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
            cv2.putText(frame, line2, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 2)
            cv2.putText(frame, "0-3=Target  E=Up  D=Down  F=Flip  Q=Quit", (10, frame_h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.imshow("Tello ArUco Tracker", frame)

        def track_fiducial(yaw_cmd, fb_cmd, lr_cmd, ud_cmd, target_found, last_target_time, current_dist):
            if ids is not None and frame is not None:
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
            
            return yaw_cmd, fb_cmd, lr_cmd, ud_cmd, target_found, last_target_time, current_dist

        self.streamon()
        time.sleep(2)
        frame_reader = self.get_frame_read()

        print("Taking off...")
        self.takeoff()
        time.sleep(3)

        dist_history = []
        lr_history = []
        vert_history = []

        last_target_time = time.time()

        try:
            while True:
                frame = frame_reader.frame
                if frame is None:
                    continue

                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                corners, ids, _ = self.aruco_data.detector.detectMarkers(gray)

                yaw_cmd = 0
                fb_cmd = 0
                lr_cmd = 0
                ud_cmd = 0
                frame_h, frame_w = frame.shape[:2]
                center_x = frame_w // 2
                center_y = frame_h // 2
                current_dist = None
                target_found = False

                yaw_cmd, fb_cmd, lr_cmd, ud_cmd, target_found, last_target_time, current_dist = track_fiducial(
                    yaw_cmd, fb_cmd, lr_cmd, ud_cmd, target_found, last_target_time, current_dist,
                )

                if not target_found and (time.time() - last_target_time) > self.flight_control.LOST_TIMEOUT:
                    yaw_cmd = 0
                    fb_cmd = 0
                    lr_cmd = 0

                 # --- Keyboard input ---
                key = '#' if variables.instructions['fingers'][0] > self.finger_thresholds[0] else cv2.waitKey(1) & 0xFF

                if key == ord('q'):
                    break
                elif key == ord('e'):
                    ud_cmd = self.flight_control.MANUAL_UD_SPEED
                elif key == ord('d'):
                    ud_cmd = -self.flight_controlMANUAL_UD_SPEED
                elif key == ord('f'):
                    self.send_rc_control(0, 0, 0, 0)
                    time.sleep(0.3)
                    try:
                        self.flip("f")
                        time.sleep(1.5)
                    except Exception as e:
                        print(f"Flip failed: {e}")
                    continue

                # Shift + 0-3: orbit search for that marker
                # Shift+0 = ')' Shift+1 = '!' Shift+2 = '@' Shift+3 = '#'
                elif key in [ord(')'), ord('!'), ord('@'), ord('#')]:
                    shift_map = {ord(')'): 0, ord('!'): 1, ord('@'): 2, ord('#'): 3}
                    orbit_target = shift_map[key]
                    print(f"[INPUT] Shift+{orbit_target} — orbiting to find ID {orbit_target}")

                    self.send_rc_control(0, 0, 0, 0)
                    time.sleep(0.3)

                    found = self.orbit_until_found(self, frame_reader, orbit_target)

                    if found:
                        self.aruco_data.TARGET_ID = orbit_target
                        clear_histories()
                        print(f"[ORBIT] Now tracking ID {self.aruco_data.TARGET_ID}")
                    else:
                        print(f"[ORBIT] ID {orbit_target} not found. Resuming ID {self.aruco_data.TARGET_ID}")

                    last_target_time = time.time()
                    continue

                # Normal 0-3: instant target switch (no orbiting)
                elif key in [ord('0'), ord('1'), ord('2'), ord('3')]:
                    new_target = key - ord('0')
                    if new_target != self.aruco_data.TARGET_ID:
                        self.aruco_data.TARGET_ID = new_target
                        clear_histories()
                        print(f"Switched to target ID: {self.aruco_data.TARGET_ID}")

                status_display()

        except KeyboardInterrupt:
            print('KeyboardInterrupt detected, shutting down.')

        finally:
            print("Landing...")
            self.send_rc_control(0, 0, 0, 0)
            time.sleep(0.5)
            self.land()
            self.streamoff()
            cv2.destroyAllWindows()
    
    def check_for_marker(self, frame_reader, target_id):
        """Grab a frame and check if the target marker is visible."""
        frame = frame_reader.frame
        if frame is None:
            return False, None
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.aruco_data.detector.detectMarkers(gray)
        if ids is not None:
            aruco.drawDetectedMarkers(frame_bgr, corners, ids)
            for corner, mid in zip(corners, ids.flatten()):
                if mid == target_id:
                    return True, frame_bgr
        return False, frame_bgr

    def orbit_until_found(self, drone, frame_reader, target_id):
        """
        Orbit around the operator using simultaneous yaw + lateral strafe.

        The idea: yaw rotates the drone's heading while lateral strafe keeps
        it moving along an arc. Since the drone is roughly facing the operator,
        yawing right + strafing right traces a clockwise circle around them.

        After each check interval we grab a frame and look for the target.
        Returns True if found, False if timeout.
        """
        print(f"[ORBIT] Searching for ID {target_id} — orbiting clockwise")

        drone.send_rc_control(0, 0, 0, 0)
        time.sleep(0.3)

        start_time = time.time()
        last_check = time.time()

        while True:
            elapsed = time.time() - start_time

            if elapsed > self.flight_control.ORBIT_TIMEOUT:
                print(f"[ORBIT] Timeout after {self.flight_control.ORBIT_TIMEOUT}s — ID {target_id} not found")
                drone.send_rc_control(0, 0, 0, 0)
                return False

            # Orbit: yaw right + strafe right = clockwise circle around centre
            # The yaw keeps the drone turning to face new directions while
            # the lateral movement sweeps it around the arc
            drone.send_rc_control(-self.flight_control.ORBIT_LATERAL_SPEED, 0, 0, self.flight_control.ORBIT_YAW_SPEED)

            # Check for marker periodically
            if time.time() - last_check >= self.flight_control.ORBIT_CHECK_INTERVAL:
                last_check = time.time()

                found, frame = self.check_for_marker(self, frame_reader, target_id)

                # Update display
                if frame is not None:
                    h, w = frame.shape[:2]
                    cv2.putText(frame,
                                f"ORBITING for ID {target_id} ({elapsed:.1f}s / {self.flight_control.ORBIT_TIMEOUT:.0f}s)",
                                (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
                    cv2.putText(frame,
                                "Yaw+strafe clockwise — looking for marker...",
                                (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 2)
                    cv2.imshow("Tello ArUco Tracker", frame)

                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    print("[ORBIT] Q pressed — aborting orbit")
                    drone.send_rc_control(0, 0, 0, 0)
                    return False
    
                if found:
                    print(f"[ORBIT] Found ID {target_id}!")
                    drone.send_rc_control(0, 0, 0, 0)
                    time.sleep(0.3)
                    return True

            time.sleep(0.05)  # ~20Hz RC update rate

if __name__ == '__main__':
    tello = FALCON()