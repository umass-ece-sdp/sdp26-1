import subprocess
from pathlib import Path
from djitellopy import Tello
import time
import threading
import numpy as np
import cv2
import cv2.aruco as aruco
from software.lib import variables
from dataclasses import dataclass, field
from threading import Thread
from collections import deque
from datetime import datetime
import os

from software.lib.fiducials import Fiducial


@dataclass
class FlightControl:
    YAW_SPEED: int = 50
    YAW_DEAD_ZONE: int = 25

    TARGET_DIST: float = 3.048
    DIST_DEAD_ZONE: float = 0.08
    FB_TIERS: list = field(default_factory=lambda: [(2.0, 100), (1.0, 75), (0.4, 45), (0.08, 20)])
    FB_MAX: int = 100

    LR_SPEED: int = 70
    LR_DEAD_ZONE: int = 20
    LR_MAX: int = 100

    VERT_SPEED: int = 75
    VERT_DEAD_ZONE: int = 20
    VERT_MAX: int = 100

    URGENCY_TIERS: list = field(default_factory=lambda: [(2.0, 1.8), (1.0, 1.4), (0.4, 1.1)])

    MANUAL_UD_SPEED: int = 50

    SMOOTH_WINDOW: int = 3
    LOST_TIMEOUT: float = 1.0

    ARC_YAW_SPEED: int = 30
    ARC_LATERAL_SPEED: int = 55
    ARC_CHECK_INTERVAL: float = 0.15
    ARC_TIMEOUT: float = 20.0
    ARC_CENTERING_TIMEOUT: float = 4.0
    ARC_CENTER_HOLD: float = 0.5
    ARC_YAW_TOL: int = 50
    ARC_DIST_TOL: float = 0.16
    ARC_REFERENCE_DIST: float = 3.048
    ARC_YAW_EXPONENT: float = 0.5
    ARC_LATERAL_MIN: int = 15
    ARC_LATERAL_MAX: int = 90
    ARC_YAW_MIN: int = 18
    ARC_YAW_MAX: int = 60
    ARC_STRAFE_ONLY_DURATION: float = 0.3

    RC_MIN_INTERVAL: float = 0.08
    RC_HEARTBEAT: float = 0.25

    VIDEO_FPS: int = 30


@dataclass
class FiducialData:
    TARGET_ID: int = 3
    VALID_TARGETS: list[int] = field(default_factory=lambda: [0, 1, 2, 3])


class DetectorThread(Thread):
    """
    Continuously detects ArUco markers on the latest frame from the drone.
    Main loop reads `.result` (a snapshot dict) under `.lock`.
    Always works on the freshest frame — skips stale ones.
    """

    def __init__(self, frame_reader, detector):
        super().__init__(daemon=True)
        self.frame_reader = frame_reader
        self.detector = detector
        self.lock = threading.Lock()
        self.running = True
        self.result = {
            'corners': None,
            'ids': None,
            'frame': None,
        }

    def run(self):
        while self.running:
            frame = self.frame_reader.frame
            if frame is None:
                time.sleep(0.005)
                continue

            if frame.shape[1] < 600:
                time.sleep(0.02)
                continue

            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = self.detector.detectMarkers(gray)

            with self.lock:
                self.result = {
                    'corners': corners,
                    'ids': ids,
                    'frame': frame_bgr,
                }

            time.sleep(0.01)

    def snapshot(self):
        with self.lock:
            return {
                'corners': self.result['corners'],
                'ids': self.result['ids'],
                'frame': self.result['frame'],
            }

    def stop(self):
        self.running = False


class VideoWriterThread(Thread):
    """
    Saves the raw RGB stream (converted to BGR) to an MP4 at VIDEO_FPS.
    Uses a simple fixed-rate timer like the old working script.
    """

    def __init__(self, frame_reader, filepath, fps):
        super().__init__(daemon=True)
        self.frame_reader = frame_reader
        self.filepath = filepath
        self.fps = fps
        self.running = True
        self.writer = None
        self.frames_written = 0

    def run(self):
        interval = 1.0 / self.fps
        next_tick = time.time()
        expected_shape = None
        min_real_w = 600

        while self.running:
            now = time.time()
            sleep_for = next_tick - now
            if sleep_for > 0:
                time.sleep(sleep_for)
            next_tick += interval
            if time.time() - next_tick > 0.5:
                next_tick = time.time() + interval

            frame = self.frame_reader.frame
            if frame is None:
                continue

            bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            h, w = bgr.shape[:2]

            if self.writer is None:
                while self.running and w < min_real_w:
                    frame = self.frame_reader.frame
                    if frame is None:
                        continue
                    bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                    h, w = bgr.shape[:2]
                if not self.running:
                    break

                fourcc = cv2.VideoWriter.fourcc(*'mp4v')
                self.writer = cv2.VideoWriter(self.filepath, fourcc, self.fps, (w, h))
                if not self.writer.isOpened():
                    print(f"[VIDEO] Failed to open writer at {self.filepath}")
                    self.running = False
                    return
                expected_shape = (h, w)
                print(f"[VIDEO] Recording {w}x{h} @ {self.fps} FPS -> {self.filepath}")

            if (h, w) != expected_shape:
                continue

            self.writer.write(bgr)
            self.frames_written += 1

    def stop(self):
        self.running = False
        time.sleep(0.1)
        if self.writer is not None:
            self.writer.release()
            print(
                f"[VIDEO] Saved {self.frames_written} frames "
                f"({self.frames_written / self.fps:.1f}s) to {self.filepath}"
            )


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

    def __init__(self, *, ssid: str, password: str):
        self.file_path = Path(__file__).parent
        self.ssid = ssid
        self.password = password

        # Thresholds for determining finger on
        self.finger_thresholds = (0.3, 0.3, 0.3, 0.3)  # V

        # Flight control and fiducial data
        self.flight_control = FlightControl()
        self.aruco_data = FiducialData()
        self.fiducial = Fiducial()
        self.commands = {   # gesture-to-command mapping
            '0000': 'quit',
            '0001': 'arc_right',
            # '0010': '',   # hard gesture to make
            '0011': 'arc_left',
            # '0100': '',   # cannot flip people off for demo
            # '0101': '',   # hard gesture to make
            '0110': 'down',
            '0111': 'up',
            '1000': 'target_1',
            '1001': 'flip',
            # '1010': '',
            # '1011': '',
            '1100': 'target_2',
            '1101': 'target_0',
            '1110': 'target_3',
            # '1111': '',
        }

        # Connect to WiFi before initializing Tello
        # self._connect_wifi()

        # Initialize normal Tello behavior
        super().__init__()

        self._last_rc = (0, 0, 0, 0)
        self._last_rc_send = 0.0

        time.sleep(3)

        # Connect to the drone and set it to SDK mode
        # Note: wait_for_state=False because some Tello units don't reliably send state packets
        # self.connect()
        self.connect(wait_for_state=False)
        time.sleep(3)

        # # Let the environment know the drone is connected
        # variables.set_drone_on()

    def _connect_wifi(self) -> None:
        '''
        Automatically configures WiFi on Linux devices by invoking
        setup_wifi.sh with the configured SSID and password.
        '''
        path_to_script = self.file_path.parent.joinpath('scripts', 'setup_wifi.ps1').as_posix()
        cmd = ['PS', path_to_script, self.ssid, self.password]

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

    def _reset_wifi(self) -> None:
        '''
        Restores Linux WiFi interfaces by invoking reset_wifi.sh.
        '''
        path_to_script = self.file_path.parent.joinpath('scripts', 'setup_wifi.ps1').as_posix()
        cmd = ['PS', path_to_script, '-Teardown']

        try:
            result = subprocess.run(cmd, check=True, capture_output=True, text=True)
            print('STDOUT:\n', result.stdout)
            print('WiFi reset successfully.')

        except subprocess.CalledProcessError as e:
            print("Command failed with exit", e.returncode)
            print('STDOUT:\n', e.stdout)
            print('STDERR:\n', e.stderr)
            print('Cannot reset WiFi configuration.')

    def _clamp(self, val, lo, hi):
        return max(lo, min(hi, val))

    def _lookup_tier(self, value, tiers, default):
        for threshold, tier_value in tiers:
            if value > threshold:
                return tier_value
        return default

    def _smooth(self, dq):
        return float(np.median(dq))

    def _should_send_rc(self, new_rc, last_rc, elapsed, heartbeat, min_interval):
        if elapsed < min_interval:
            return False
        return new_rc != last_rc or elapsed > heartbeat

    def _hud(self, frame, text, pos, color):
        cv2.putText(frame, text, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

    def _pose_from_corner(self, corner):
        success, _, tvec = cv2.solvePnP(
            self.fiducial.obj_points,
            corner.reshape(4, 2),
            self.fiducial.camera_matrix,
            self.fiducial.dist_coeffs,
        )
        if not success:
            return None
        return np.linalg.norm(tvec.ravel()[:3]), tvec

    def _send_rc_throttled(self, lr, fb, ud, yaw):
        now = time.time()
        new_rc = (lr, fb, ud, yaw)
        elapsed = now - self._last_rc_send
        if self._should_send_rc(
            new_rc,
            self._last_rc,
            elapsed,
            self.flight_control.RC_HEARTBEAT,
            self.flight_control.RC_MIN_INTERVAL,
        ):
            self.send_rc_control(lr, fb, ud, yaw)
            self._last_rc = new_rc
            self._last_rc_send = now

    def _stop_and_reset_rc_state(self):
        self.send_rc_control(0, 0, 0, 0)
        self._last_rc = (0, 0, 0, 0)
        self._last_rc_send = time.time()

    def _read_glove_gesture(self, fingers: tuple[float, float, float, float]) -> str | None:
        """
        Interpret the current glove sensor values as a high-level control gesture.

        Contract:
        - Input:
          - `fingers`: 4-tuple of floats from `variables.read_instr()['fingers']`.
          - Order is fixed and corresponds to the four stretch sensors on the glove,
            one value per finger channel.
          - `self.finger_thresholds` is available as a 4-tuple of per-finger thresholds
            that an implementation may use when classifying gestures.

        - Output:
          Return exactly one of the following string literals, or `None`:
          - `'arc_left'`: caller must trigger
            `self._arc_to_next_fiducial(..., direction='left')`.
          - `'arc_right'`: caller must trigger
            `self._arc_to_next_fiducial(..., direction='right')`.
          - `'up'`: caller must set `ud_cmd = +self.flight_control.MANUAL_UD_SPEED`.
          - `'down'`: caller must set `ud_cmd = -self.flight_control.MANUAL_UD_SPEED`.
          - `'flip'`: caller must trigger `self.flip('f')`.
          - `'quit'`: caller must initiate landing and shutdown.
          - `'target_0'`, `'target_1'`, `'target_2'`, `'target_3'`: caller must switch
            `self.aruco_data.TARGET_ID` to the corresponding integer.
          - `None`: no gesture detected this frame; caller takes no gesture action.

        Timing and state expectations:
        - This method is called once per main loop iteration.
        - The method is stateless: it only maps the current `fingers` values to a
          gesture token and must not perform debouncing or edge detection.
        - Any one-shot behavior (e.g., `'flip'`, `'arc_left'`, `'arc_right'`) must be
          debounced by the caller, not inside this method.
        """
        
        # Compare fingers against thresholds to get states, 0 if active 1 if not
        states = ''.join(['0' if f > t else '1' for f, t in zip(fingers, self.finger_thresholds)])

        if states in self.commands:
            return self.commands[states]
        return None

    def _arc_to_next_fiducial(self, frame_reader, current_target_id: int, direction: str) -> int | None:
        """
        Arc around the operator in the given direction ('left' or 'right')
        until any fiducial OTHER than current_target_id is detected, then
        center on it (yaw + distance) and return its ID.

        Returns:
          new_target_id (int) if successful
          None if timed out or centering failed
        """
        if direction not in ('left', 'right'):
            print(f"[ARC] Invalid direction: {direction}")
            return None

        if direction == 'right':
            yaw_sign = 1
            lr_sign = -1
        else:
            yaw_sign = -1
            lr_sign = 1

        print(f"[ARC] Arcing {direction} from ID {current_target_id} to find next fiducial")

        self.send_rc_control(0, 0, 0, 0)
        time.sleep(0.3)

        start_time = time.time()
        last_check = 0.0
        last_known_dist = self.flight_control.TARGET_DIST
        new_id = None

        ratio = max(self.flight_control.TARGET_DIST, 0.3) / self.flight_control.ARC_REFERENCE_DIST
        base_lateral = int(
            self._clamp(
                self.flight_control.ARC_LATERAL_SPEED * ratio,
                self.flight_control.ARC_LATERAL_MIN,
                self.flight_control.ARC_LATERAL_MAX,
            )
        )
        base_yaw = int(
            self._clamp(
                self.flight_control.ARC_YAW_SPEED * (ratio ** self.flight_control.ARC_YAW_EXPONENT),
                self.flight_control.ARC_YAW_MIN,
                self.flight_control.ARC_YAW_MAX,
            )
        )
        print(
            f"[ARC] Speeds for TARGET_DIST={self.flight_control.TARGET_DIST:.2f}m: "
            f"lateral={base_lateral} yaw={base_yaw}"
        )

        last_arc_rc = None
        last_arc_send = 0.0

        while True:
            elapsed = time.time() - start_time
            if elapsed > self.flight_control.ARC_TIMEOUT:
                print(f"[ARC] Timeout after {self.flight_control.ARC_TIMEOUT}s — no new fiducial")
                self.send_rc_control(0, 0, 0, 0)
                return None

            ratio = self.flight_control.TARGET_DIST / max(last_known_dist, 0.3)
            lr_speed = int(
                self._clamp(
                    base_lateral * ratio,
                    self.flight_control.ARC_LATERAL_MIN,
                    self.flight_control.ARC_LATERAL_MAX,
                )
            )
            if elapsed < self.flight_control.ARC_STRAFE_ONLY_DURATION:
                yaw_now = 0
            else:
                yaw_now = yaw_sign * base_yaw
            new_rc = (lr_sign * lr_speed, 0, 0, yaw_now)

            now = time.time()
            elapsed_arc = now - last_arc_send
            if self._should_send_rc(new_rc, last_arc_rc, elapsed_arc, 0.25, 0.08):
                self.send_rc_control(*new_rc)
                last_arc_rc = new_rc
                last_arc_send = now

            if time.time() - last_check >= self.flight_control.ARC_CHECK_INTERVAL:
                last_check = time.time()
                frame = frame_reader.frame
                display_bgr = None
                current_dist_seen = None

                if frame is not None and frame.shape[1] >= 600:
                    display_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                    gray = cv2.cvtColor(display_bgr, cv2.COLOR_BGR2GRAY)
                    corners, ids, _ = self.fiducial.detector.detectMarkers(gray)

                    if ids is not None and corners is not None:
                        aruco.drawDetectedMarkers(display_bgr, corners, ids)
                        for corner, marker_id in zip(corners, ids.flatten()):
                            pose = self._pose_from_corner(corner)
                            if pose is None:
                                continue
                            dist, _ = pose
                            if current_dist_seen is None or dist < current_dist_seen:
                                current_dist_seen = dist

                            if int(marker_id) != current_target_id and new_id is None:
                                new_id = int(marker_id)

                    if current_dist_seen is not None:
                        last_known_dist = current_dist_seen

                if display_bgr is not None:
                    dist_str = f"{last_known_dist:.2f}m"
                    cv2.putText(
                        display_bgr,
                        f"ARC {direction.upper()} from ID {current_target_id} "
                        f"({elapsed:.1f}s / {self.flight_control.ARC_TIMEOUT:.0f}s)",
                        (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 165, 255),
                        2,
                    )
                    cv2.putText(
                        display_bgr,
                        f"Dist:{dist_str}  Looking for new fiducial...",
                        (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 200, 255),
                        2,
                    )
                    cv2.imshow("Tello ArUco Tracker", display_bgr)
                    cv2.waitKey(1)

                if new_id is not None:
                    print(f"[ARC] Found new fiducial ID {new_id} — centering")
                    break

            time.sleep(0.03)

        self.send_rc_control(0, 0, 0, 0)
        time.sleep(0.2)

        center_start = time.time()
        in_tolerance_since = None

        while True:
            elapsed = time.time() - center_start
            if elapsed > self.flight_control.ARC_CENTERING_TIMEOUT:
                print("[ARC] Centering timeout — handing off anyway")
                self.send_rc_control(0, 0, 0, 0)
                return new_id

            frame = frame_reader.frame
            if frame is None or frame.shape[1] < 600:
                time.sleep(0.02)
                continue

            display_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            gray = cv2.cvtColor(display_bgr, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = self.fiducial.detector.detectMarkers(gray)

            found_new = False
            yaw_err_px = 0
            dist_err = 0.0

            if ids is not None and corners is not None:
                aruco.drawDetectedMarkers(display_bgr, corners, ids)
                for corner, marker_id in zip(corners, ids.flatten()):
                    if int(marker_id) != new_id:
                        continue
                    pose = self._pose_from_corner(corner)
                    if pose is None:
                        continue
                    found_new = True
                    dist, _ = pose
                    dist_err = dist - self.flight_control.TARGET_DIST

                    _, frame_w = display_bgr.shape[:2]
                    marker_cx = int(corner[0, :, 0].mean())
                    yaw_err_px = marker_cx - (frame_w // 2)
                    break

            if not found_new:
                self.send_rc_control(0, 0, 0, yaw_sign * 15)
                in_tolerance_since = None
                cv2.putText(
                    display_bgr,
                    f"CENTERING ID {new_id} — lost, searching",
                    (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 100, 255),
                    2,
                )
                cv2.imshow("Tello ArUco Tracker", display_bgr)
                cv2.waitKey(1)
                time.sleep(0.05)
                continue

            yaw_cmd = 0
            fb_cmd = 0

            if abs(yaw_err_px) > self.flight_control.YAW_DEAD_ZONE:
                yaw_cmd = int(
                    self._clamp(
                        self.flight_control.YAW_SPEED * (yaw_err_px / (display_bgr.shape[1] / 2)),
                        -100,
                        100,
                    )
                )

            abs_d = abs(dist_err)
            fb_speed = self._lookup_tier(abs_d, self.flight_control.FB_TIERS, 0)
            if fb_speed > 0:
                sign = 1 if dist_err > 0 else -1
                fb_cmd = int(self._clamp(sign * fb_speed, -self.flight_control.FB_MAX, self.flight_control.FB_MAX))

            self.send_rc_control(0, fb_cmd, 0, yaw_cmd)

            in_tol = (
                abs(yaw_err_px) <= self.flight_control.ARC_YAW_TOL
                and abs_d <= self.flight_control.ARC_DIST_TOL
            )
            if in_tol:
                if in_tolerance_since is None:
                    in_tolerance_since = time.time()
                elif time.time() - in_tolerance_since >= self.flight_control.ARC_CENTER_HOLD:
                    print(f"[ARC] Centered on ID {new_id} — handing off")
                    self.send_rc_control(0, 0, 0, 0)
                    return new_id
            else:
                in_tolerance_since = None

            self._hud(
                display_bgr,
                f"CENTERING ID {new_id}  yaw_err:{yaw_err_px:+d}px  dist_err:{dist_err:+.2f}m",
                (10, 25),
                (0, 200, 100) if in_tol else (0, 165, 255),
            )
            cv2.imshow("Tello ArUco Tracker", display_bgr)
            cv2.waitKey(1)

            time.sleep(0.03)

    def track_target(self):
        battery = self.get_battery()
        print(f"Battery: {battery}%")

        if battery < 15:
            print("Battery too low to fly. Exiting.")
            return

        self.streamon()
        time.sleep(2)

        frame_reader = self.get_frame_read()

        det_thread = DetectorThread(frame_reader, self.fiducial.detector)
        det_thread.start()

        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        video_path = os.path.join(str(self.file_path), f"tello_flight_{timestamp}.mp4")
        video_thread = VideoWriterThread(frame_reader, video_path, self.flight_control.VIDEO_FPS)
        video_thread.start()

        print("Taking off...")
        self.takeoff()
        time.sleep(3)

        dist_history = deque(maxlen=self.flight_control.SMOOTH_WINDOW)
        lr_history = deque(maxlen=self.flight_control.SMOOTH_WINDOW)
        vert_history = deque(maxlen=self.flight_control.SMOOTH_WINDOW)

        last_target_time = time.time()
        last_gesture = None

        try:
            while True:
                snap = det_thread.snapshot()
                frame = snap['frame']

                if frame is None:
                    time.sleep(0.01)
                    continue

                frame = frame.copy()
                corners = snap['corners']
                ids = snap['ids']

                yaw_cmd = 0
                fb_cmd = 0
                lr_cmd = 0
                ud_cmd = 0

                frame_h, frame_w = frame.shape[:2]
                center_x = frame_w // 2
                center_y = frame_h // 2
                current_dist = None
                target_found = False

                if ids is not None and corners is not None:
                    aruco.drawDetectedMarkers(frame, corners, ids)

                    for corner, marker_id in zip(corners, ids.flatten()):
                        pose = self._pose_from_corner(corner)
                        if pose is None:
                            continue
                        current_marker_dist, tvec = pose

                        success, rvec, _ = cv2.solvePnP(
                            self.fiducial.obj_points,
                            corner.reshape(4, 2),
                            self.fiducial.camera_matrix,
                            self.fiducial.dist_coeffs,
                        )
                        if success:
                            cv2.drawFrameAxes(
                                frame,
                                self.fiducial.camera_matrix,
                                self.fiducial.dist_coeffs,
                                rvec,
                                tvec,
                                0.05,
                            )

                        label = f"ID {marker_id}: x={tvec[0, 0]:.2f} y={tvec[1, 0]:.2f} z={tvec[2, 0]:.2f}m"
                        pos = tuple(corner[0][0].astype(int))
                        cv2.putText(frame, label, (pos[0], pos[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                        if int(marker_id) == self.aruco_data.TARGET_ID:
                            target_found = True
                            last_target_time = time.time()

                            marker_cx = int(corner[0, :, 0].mean())
                            marker_cy = int(corner[0, :, 1].mean())

                            cv2.circle(frame, (marker_cx, marker_cy), 8, (0, 0, 255), 2)
                            cv2.line(frame, (center_x - 15, center_y), (center_x + 15, center_y), (255, 0, 0), 1)
                            cv2.line(frame, (center_x, center_y - 15), (center_x, center_y + 15), (255, 0, 0), 1)

                            error_x = marker_cx - center_x
                            if abs(error_x) > self.flight_control.YAW_DEAD_ZONE:
                                yaw_cmd = int(self._clamp(self.flight_control.YAW_SPEED * (error_x / (frame_w / 2)), -100, 100))

                            dist_history.append(current_marker_dist)
                            current_dist = self._smooth(dist_history)

                            dist_error = current_dist - self.flight_control.TARGET_DIST
                            abs_err = abs(dist_error)

                            fb_speed = self._lookup_tier(abs_err, self.flight_control.FB_TIERS, 0)
                            if fb_speed > 0:
                                sign = 1 if dist_error > 0 else -1
                                fb_cmd = int(self._clamp(sign * fb_speed, -self.flight_control.FB_MAX, self.flight_control.FB_MAX))

                            urgency = self._lookup_tier(abs_err, self.flight_control.URGENCY_TIERS, 1.0)

                            lr_history.append(float(tvec[0, 0]))
                            smooth_x = self._smooth(lr_history)
                            lr_error = smooth_x * (self.fiducial.camera_matrix[0, 0] / current_dist) if current_dist else 0
                            if abs(lr_error) > self.flight_control.LR_DEAD_ZONE:
                                lr_cmd = int(
                                    self._clamp(
                                        self.flight_control.LR_SPEED * urgency * (lr_error / (frame_w / 2)),
                                        -self.flight_control.LR_MAX,
                                        self.flight_control.LR_MAX,
                                    )
                                )

                            error_y = marker_cy - center_y
                            vert_history.append(error_y)
                            smooth_vert = self._smooth(vert_history)
                            if abs(smooth_vert) > self.flight_control.VERT_DEAD_ZONE:
                                ud_cmd = int(
                                    self._clamp(
                                        -self.flight_control.VERT_SPEED * urgency * (smooth_vert / (frame_h / 2)),
                                        -self.flight_control.VERT_MAX,
                                        self.flight_control.VERT_MAX,
                                    )
                                )

                if not target_found and (time.time() - last_target_time) > self.flight_control.LOST_TIMEOUT:
                    yaw_cmd, fb_cmd, lr_cmd = 0, 0, 0

                instr = variables.read_instr()
                fingers = instr.get('fingers', (0.0, 0.0, 0.0, 0.0))

                gesture = None
                try:
                    gesture = self._read_glove_gesture(fingers)
                except NotImplementedError:
                    gesture = None

                if gesture != last_gesture and gesture is not None:
                    if gesture == 'quit':
                        break

                    if gesture == 'up':
                        ud_cmd = self.flight_control.MANUAL_UD_SPEED
                    elif gesture == 'down':
                        ud_cmd = -self.flight_control.MANUAL_UD_SPEED
                    elif gesture == 'flip':
                        self._stop_and_reset_rc_state()
                        time.sleep(0.3)
                        try:
                            self.flip('f')
                            time.sleep(1.5)
                        except Exception as e:
                            print(f"Flip failed: {e}")
                        last_gesture = gesture
                        continue
                    elif gesture in ('arc_left', 'arc_right'):
                        direction = 'left' if gesture == 'arc_left' else 'right'

                        self._stop_and_reset_rc_state()
                        time.sleep(0.2)

                        new_id = self._arc_to_next_fiducial(
                            frame_reader,
                            self.aruco_data.TARGET_ID,
                            direction,
                        )

                        self._stop_and_reset_rc_state()

                        if new_id is not None:
                            self.aruco_data.TARGET_ID = new_id
                            dist_history.clear()
                            lr_history.clear()
                            vert_history.clear()
                            print(f"[ARC] Now tracking ID {self.aruco_data.TARGET_ID}")
                        else:
                            print(f"[ARC] No handoff — resuming ID {self.aruco_data.TARGET_ID}")

                        last_target_time = time.time()
                        last_gesture = gesture
                        continue
                    elif gesture in ('target_0', 'target_1', 'target_2', 'target_3'):
                        new_target = int(gesture[-1])
                        if new_target != self.aruco_data.TARGET_ID:
                            self.aruco_data.TARGET_ID = new_target
                            dist_history.clear()
                            lr_history.clear()
                            vert_history.clear()
                            print(f"Switched to target ID: {self.aruco_data.TARGET_ID}")

                last_gesture = gesture

                dist_str = f"{current_dist:.2f}m" if current_dist else "N/A"
                if current_dist is not None:
                    fb_tier = self._lookup_tier(abs(current_dist - self.flight_control.TARGET_DIST), self.flight_control.FB_TIERS, 0)
                    urgency = self._lookup_tier(abs(current_dist - self.flight_control.TARGET_DIST), self.flight_control.URGENCY_TIERS, 1.0)
                    tier_str = f"  TIER:{fb_tier} URG:{urgency:.1f}x"
                else:
                    tier_str = ""

                line1 = (
                    f"TARGET:{self.aruco_data.TARGET_ID}  "
                    f"YAW:{yaw_cmd:+d} FB:{fb_cmd:+d} LR:{lr_cmd:+d} UD:{ud_cmd:+d}"
                )
                line2 = f"DIST:{dist_str} TARGET:{self.flight_control.TARGET_DIST:.2f}m{tier_str}"
                lost_str = "" if target_found else " [TARGET LOST]"

                self._hud(frame, line1 + lost_str, (10, 25), (0, 0, 255))
                self._hud(frame, line2, (10, 50), (0, 200, 255))
                cv2.putText(frame, "REC", (frame_w - 60, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                cv2.circle(frame, (frame_w - 75, 24), 6, (0, 0, 255), -1)
                cv2.putText(
                    frame,
                    "Glove control active",
                    (10, frame_h - 15),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.42,
                    (255, 255, 255),
                    1,
                )

                self._send_rc_throttled(lr_cmd, fb_cmd, ud_cmd, yaw_cmd)

                cv2.imshow("Tello ArUco Tracker", frame)
                cv2.waitKey(1)

        finally:
            print("Landing...")
            try:
                self.send_rc_control(0, 0, 0, 0)
                time.sleep(0.5)
                self.land()
            except Exception as e:
                print(f"Land error: {e}")

            print("Stopping detector thread...")
            det_thread.stop()
            det_thread.join(timeout=2.0)

            print("Stopping video writer...")
            video_thread.stop()
            video_thread.join(timeout=3.0)

            try:
                self.streamoff()
            except Exception as e:
                print(f"Streamoff error: {e}")

            cv2.destroyAllWindows()
            print("Done.")


if __name__ == '__main__':
    tello = FALCON(ssid="TELLO-FE046A", password="")
    tello.track_target()
