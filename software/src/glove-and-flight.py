'''
Tello ArUco tracker with glove gesture control.

Gestures (finger pattern, 1 = finger UP/extended, 0 = finger bent/down):
  1000  -> Arc LEFT  (only index up)
  1100  -> Arc RIGHT (index + middle up)
  1110  -> QUIT      (index + middle + ring up; pinky down)

Sensor convention:
  Voltage RISES when a finger is bent down.
  So finger is UP (1) when voltage < that finger's threshold.

Left-hand glove packet order: [pinky, ring, middle, index]
  fingers[0]=pinky, fingers[1]=ring, fingers[2]=middle, fingers[3]=index
FINGER_ORDER below maps packet positions to pattern positions so the
pattern string reads [index, middle, ring, pinky] left-to-right.

Keyboard fallback preserved:
  Q       quit
  0-3     switch target marker
  E/D     manual up/down
  F       flip
  A/S     arc left/right (same as gestures, for debugging without glove)

Threads:
  - Detector: ArUco detection on latest frame
  - Video writer: saves raw stream to MP4
  - Glove client: reads TCP packets, decodes gestures, queues events
'''

from djitellopy import Tello
import cv2
import cv2.aruco as aruco
import numpy as np
import time
import os
import socket
import struct
import threading
import queue
import platform
from datetime import datetime

# =====================================================================
#  Config
# =====================================================================

# --- Debug ---
GROUND_TEST = False    # True = no takeoff / no RC sends, just stream + detect
GLOVE_DEBUG = True     # Print finger voltages + recognized gestures
GLOVE_ENABLED = True   # If False, runs keyboard-only (no glove socket)
CALIBRATE_THRESHOLDS = True # True to calibrate thresholds before starting

# --- Glove ---
GLOVE_HOST = '192.168.4.1'
GLOVE_PORT = 5000
GLOVE_RECONNECT_DELAY = 2.0
GLOVE_PACKET_SIZE = 24       # 6 little-endian floats
GLOVE_RECV_TIMEOUT = 3.0
GLOVE_CONNECT_TIMEOUT = 5.0

# Per-finger threshold (volts). Voltage RISES as the finger bends down.
#   voltage <  threshold -> finger UP   (pattern bit = 1)
#   voltage >= threshold -> finger DOWN (pattern bit = 0)
# Indexed by RAW packet index for the left-hand glove:
#   [0]=pinky, [1]=ring, [2]=middle, [3]=index
FINGER_UP_THRESHOLD = [
    0.20,   # [0] pinky
    0.23,   # [1] ring
    0.27,   # [2] middle
    0.29,   # [3] index
]

# Maps packet index -> pattern position (leftmost first).
# Pattern reads [index, middle, ring, pinky] for natural left-to-right
# reading on a left hand, so pattern[0] (leftmost) = packet[3] (index).
FINGER_ORDER = [3, 2, 1, 0]

# Gesture must be held continuously this long before firing (debounce)
GESTURE_HOLD_TIME = 0.4      # seconds
# After firing, ignore further gestures for this long
GESTURE_COOLDOWN = 1.5       # seconds

# Pattern -> action. Patterns are 4-char strings, leftmost = FINGER_ORDER[0].
GESTURE_MAP = {
    '1000': 'arc_left',
    '1100': 'arc_right',
    '0000': 'quit',
    '0100': 'flip',
    '1001': 'up',
    '0011': 'down',
    '1010': 'further',
    '0110': 'closer',
}

# --- ArUco / tracking ---
MARKER_SIZE_DEFAULT = 0.2032
SHOULDER_MARKER_SIZE = 0.127
ARUCO_DICT = aruco.DICT_4X4_50
TARGET_ID = 0
VALID_TARGETS = [0, 1, 2, 3]

# --- Control tuning ---
YAW_SPEED = 50
YAW_DEAD_ZONE = 25

TARGET_DISTS = [3.048, 6.096, 9.144, 12.192]
TARGET_DIST_INDEX = 0
TARGET_DIST = TARGET_DISTS[TARGET_DIST_INDEX]
DIST_DEAD_ZONE = 0.08

FB_TIERS = [
    (2.0, 100),
    (1.0,  75),
    (0.4,  45),
    (0.08, 20),
]
FB_MAX = 100

LR_SPEED = 70
LR_DEAD_ZONE = 20
LR_MAX = 100

VERT_SPEED = 75
VERT_DEAD_ZONE = 20
VERT_MAX = 100

URGENCY_TIERS = [
    (2.0, 1.8),
    (1.0, 1.4),
    (0.4, 1.1),
]

MANUAL_UD_SPEED = 50
SMOOTH_WINDOW = 3

# --- Arc-to-next-fiducial config ---
ARC_YAW_SPEED = 25
ARC_LATERAL_SPEED = 45
ARC_CHECK_INTERVAL = 0.15
ARC_TIMEOUT = 20.0
ARC_CENTERING_TIMEOUT = 4.0
ARC_CENTER_HOLD = 0.5
ARC_YAW_TOL = YAW_DEAD_ZONE * 2
ARC_DIST_TOL = DIST_DEAD_ZONE * 2

ARC_REFERENCE_DIST = 3.048
ARC_YAW_EXPONENT = 0.5
ARC_LATERAL_MIN = 15
ARC_LATERAL_MAX = 90
ARC_YAW_MIN = 18
ARC_YAW_MAX = 60
ARC_STRAFE_ONLY_DURATION = 0.3

# --- RC throttling ---
RC_MIN_INTERVAL = 0.08
RC_HEARTBEAT = 0.25

# --- Video recording ---
VIDEO_FPS = 30
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# Detect OS for video codec selection
OS_TYPE = platform.system()
if OS_TYPE == 'Windows':
    VIDEO_CODEC = 'mp4v'
    VIDEO_EXT = '.mp4'
else:  # Linux, macOS, etc.
    VIDEO_CODEC = 'XVID'
    VIDEO_EXT = '.avi'


# =====================================================================
#  ArUco setup
# =====================================================================
aruco_dict = aruco.getPredefinedDictionary(ARUCO_DICT)
detector_params = aruco.DetectorParameters()
detector_params.cornerRefinementMethod = aruco.CORNER_REFINE_NONE
detector = aruco.ArucoDetector(aruco_dict, detector_params)

half_default = MARKER_SIZE_DEFAULT / 2
OBJ_POINTS_DEFAULT = np.array([
    [-half_default,  half_default, 0],
    [ half_default,  half_default, 0],
    [ half_default, -half_default, 0],
    [-half_default, -half_default, 0],
], dtype=np.float32)

half_shoulder = SHOULDER_MARKER_SIZE / 2
OBJ_POINTS_SHOULDER = np.array([
    [-half_shoulder,  half_shoulder, 0],
    [ half_shoulder,  half_shoulder, 0],
    [ half_shoulder, -half_shoulder, 0],
    [-half_shoulder, -half_shoulder, 0],
], dtype=np.float32)

camera_matrix = np.array([
    [921.170702, 0.000000, 459.904354],
    [0.000000, 919.018377, 351.238301],
    [0.000000, 0.000000, 1.000000]
], dtype=np.float64)
dist_coeffs = np.array(
    [-0.033458, 0.105152, 0.001256, -0.006647, 0.000000],
    dtype=np.float64
)


# =====================================================================
#  Helpers
# =====================================================================
def clamp(val, lo, hi):
    return max(lo, min(hi, val))


def fb_speed_from_error(abs_err):
    for threshold, speed in FB_TIERS:
        if abs_err > threshold:
            return speed
    return 0


def urgency_from_error(abs_err):
    for threshold, mult in URGENCY_TIERS:
        if abs_err > threshold:
            return mult
    return 1.0


def arc_speeds_for_distance(target_dist):
    ratio = max(target_dist, 0.3) / ARC_REFERENCE_DIST
    lateral = int(clamp(ARC_LATERAL_SPEED * ratio,
                        ARC_LATERAL_MIN, ARC_LATERAL_MAX))
    yaw = int(clamp(ARC_YAW_SPEED * (ratio ** ARC_YAW_EXPONENT),
                    ARC_YAW_MIN, ARC_YAW_MAX))
    return lateral, yaw


def clear_histories(dist_history, lr_history, vert_history):
    dist_history.clear()
    lr_history.clear()
    vert_history.clear()


# =====================================================================
#  Detector thread
# =====================================================================
class DetectorThread(threading.Thread):
    def __init__(self, frame_reader):
        super().__init__(daemon=True)
        self.frame_reader = frame_reader
        self.lock = threading.Lock()
        self.running = True
        self.result = {
            'corners': None,
            'ids': None,
            'frame': None,
            'timestamp': 0.0,
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
            corners, ids, _ = detector.detectMarkers(gray)

            with self.lock:
                self.result = {
                    'corners': corners,
                    'ids': ids,
                    'frame': frame_bgr,
                    'timestamp': time.time(),
                }
            time.sleep(0.01)

    def snapshot(self):
        with self.lock:
            return dict(self.result)

    def stop(self):
        self.running = False


# =====================================================================
#  Video writer thread
# =====================================================================
class VideoWriterThread(threading.Thread):
    def __init__(self, frame_reader, filepath):
        super().__init__(daemon=True)
        self.frame_reader = frame_reader
        self.filepath = filepath
        self.running = True
        self.writer = None
        self.frames_written = 0

    def run(self):
        interval = 1.0 / VIDEO_FPS
        next_tick = time.time()
        expected_shape = None
        stable_count = 0
        STABLE_REQUIRED = 5
        MIN_REAL_W = 600

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
                if w < MIN_REAL_W:
                    continue
                stable_count += 1
                if stable_count < STABLE_REQUIRED:
                    continue

                fourcc = cv2.VideoWriter_fourcc(*VIDEO_CODEC)
                self.writer = cv2.VideoWriter(
                    self.filepath, fourcc, VIDEO_FPS, (w, h)
                )
                if not self.writer.isOpened():
                    print(f"[VIDEO] Failed to open writer at {self.filepath}")
                    self.running = False
                    return
                expected_shape = (h, w)
                print(f"[VIDEO] Recording {w}x{h} @ {VIDEO_FPS} FPS -> {self.filepath}")

            if (h, w) != expected_shape:
                continue

            self.writer.write(bgr)
            self.frames_written += 1

    def stop(self):
        self.running = False
        time.sleep(0.1)
        if self.writer is not None:
            self.writer.release()
            print(f"[VIDEO] Saved {self.frames_written} frames "
                  f"({self.frames_written / VIDEO_FPS:.1f}s) to {self.filepath}")


# =====================================================================
#  Glove client thread
# =====================================================================
def _recv_exact(sock, n):
    """Read exactly n bytes; loop over short reads. None on close/timeout."""
    buf = bytearray()
    while len(buf) < n:
        try:
            chunk = sock.recv(n - len(buf))
        except socket.timeout:
            return None
        if not chunk:
            return None
        buf.extend(chunk)
    return bytes(buf)


def _connect_glove(stop_event):
    """Connect to glove with retry. Returns socket or None if stopped."""
    while not stop_event.is_set():
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(GLOVE_CONNECT_TIMEOUT)
        try:
            print(f"[GLOVE] Connecting to {GLOVE_HOST}:{GLOVE_PORT}...")
            sock.connect((GLOVE_HOST, GLOVE_PORT))
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.settimeout(GLOVE_RECV_TIMEOUT)
            print("[GLOVE] Connected!")
            return sock
        except (socket.error, OSError) as e:
            print(f"[GLOVE] Connection failed: {e}")
            try:
                sock.close()
            except Exception:
                pass
            slept = 0.0
            while slept < GLOVE_RECONNECT_DELAY:
                if stop_event.is_set():
                    return None
                time.sleep(0.1)
                slept += 0.1
    return None


def _fingers_to_pattern(fingers):
    """
    Convert raw finger voltages to a 4-char pattern string per FINGER_ORDER.
    Voltage rises when a finger bends down, so:
       '1' = finger UP   (voltage < that finger's threshold)
       '0' = finger DOWN (voltage >= that finger's threshold)
    """
    bits = []
    for idx in FINGER_ORDER:
        bits.append('1' if fingers[idx] < FINGER_UP_THRESHOLD[idx] else '0')
    return ''.join(bits)


class GloveThread(threading.Thread):
    """
    Reads packets from the glove, debounces gestures, and pushes
    recognized gesture events ('arc_left', 'arc_right', 'quit') into
    the event queue.
    """
    def __init__(self, event_queue, stop_event):
        super().__init__(daemon=True)
        self.event_queue = event_queue
        self.stop_event = stop_event
        self.connected = False
        self.last_fingers = (0.0, 0.0, 0.0, 0.0)
        self.last_pattern = '0000'

    def run(self):
        sock = _connect_glove(self.stop_event)
        if sock is None:
            return
        self.connected = True

        # Debounce state
        candidate_pattern = None
        candidate_since = 0.0
        last_fired_at = 0.0

        while not self.stop_event.is_set():
            packet = _recv_exact(sock, GLOVE_PACKET_SIZE)
            if packet is None:
                print("[GLOVE] Lost connection - reconnecting...")
                self.connected = False
                try:
                    sock.close()
                except Exception:
                    pass
                sock = _connect_glove(self.stop_event)
                if sock is None:
                    return
                self.connected = True
                candidate_pattern = None
                continue

            # ACK back to firmware
            try:
                sock.sendall(b'A')
            except (socket.error, OSError):
                continue

            try:
                data = struct.unpack('<ffffff', packet)
            except struct.error:
                continue

            fingers = data[0:4]
            self.last_fingers = fingers
            pattern = _fingers_to_pattern(fingers)
            self.last_pattern = pattern

            if GLOVE_DEBUG:
                print(f"[GLOVE] V={fingers[0]:.2f},{fingers[1]:.2f},"
                      f"{fingers[2]:.2f},{fingers[3]:.2f}  pattern={pattern}")

            now = time.time()

            # Cooldown: ignore everything for a bit after firing
            if now - last_fired_at < GESTURE_COOLDOWN:
                candidate_pattern = None
                continue

            # Only patterns we care about
            if pattern not in GESTURE_MAP:
                candidate_pattern = None
                continue

            # Debounce: hold the same pattern for GESTURE_HOLD_TIME
            if pattern != candidate_pattern:
                candidate_pattern = pattern
                candidate_since = now
                continue

            if now - candidate_since < GESTURE_HOLD_TIME:
                continue

            # Fire
            action = GESTURE_MAP[pattern]
            print(f"[GLOVE] Gesture {pattern} -> {action}")
            try:
                self.event_queue.put_nowait(action)
            except queue.Full:
                pass
            last_fired_at = now
            candidate_pattern = None  # require release before re-firing


# =====================================================================
#  Arc-to-next-fiducial
# =====================================================================
def arc_to_next_fiducial(drone, frame_reader, current_target_id, direction):
    if direction not in ('left', 'right'):
        print(f"[ARC] Invalid direction: {direction}")
        return None

    if direction == 'right':
        yaw_sign = 1
        lr_sign = -1
    else:
        yaw_sign = -1
        lr_sign = 1

    print(f"[ARC] Arcing {direction} from ID {current_target_id}")

    drone.send_rc_control(0, 0, 0, 0)
    time.sleep(0.3)

    start_time = time.time()
    last_check = 0.0
    last_known_dist = TARGET_DIST
    new_id = None

    base_lateral, base_yaw = arc_speeds_for_distance(TARGET_DIST)
    print(f"[ARC] Speeds: lateral={base_lateral} yaw={base_yaw}")

    last_arc_rc = None
    last_arc_send = 0.0
    ARC_RC_INTERVAL = 0.08

    while True:
        elapsed = time.time() - start_time
        if elapsed > ARC_TIMEOUT:
            print("[ARC] Timeout - no new fiducial")
            drone.send_rc_control(0, 0, 0, 0)
            return None

        ratio = TARGET_DIST / max(last_known_dist, 0.3)
        lr_speed = int(clamp(base_lateral * ratio,
                             ARC_LATERAL_MIN, ARC_LATERAL_MAX))
        if elapsed < ARC_STRAFE_ONLY_DURATION:
            yaw_now = 0
        else:
            yaw_now = yaw_sign * base_yaw
        new_rc = (lr_sign * lr_speed, 0, 0, yaw_now)

        now = time.time()
        if new_rc != last_arc_rc or (now - last_arc_send) > 0.25:
            if (now - last_arc_send) >= ARC_RC_INTERVAL:
                drone.send_rc_control(*new_rc)
                last_arc_rc = new_rc
                last_arc_send = now

        if time.time() - last_check >= ARC_CHECK_INTERVAL:
            last_check = time.time()
            frame = frame_reader.frame
            display_bgr = None
            current_dist_seen = None

            if frame is not None and frame.shape[1] >= 600:
                display_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                gray = cv2.cvtColor(display_bgr, cv2.COLOR_BGR2GRAY)
                corners, ids, _ = detector.detectMarkers(gray)

                if ids is not None:
                    aruco.drawDetectedMarkers(display_bgr, corners, ids)
                    for corner, mid in zip(corners, ids.flatten()):
                        current_obj_points = OBJ_POINTS_SHOULDER if mid in (1, 2) else OBJ_POINTS_DEFAULT
                        success, rvec, tvec = cv2.solvePnP(
                            current_obj_points, corner.reshape(4, 2),
                            camera_matrix, dist_coeffs
                        )
                        if not success:
                            continue
                        x, y, z = tvec[0, 0], tvec[1, 0], tvec[2, 0]
                        d = np.sqrt(x**2 + y**2 + z**2)
                        if current_dist_seen is None or d < current_dist_seen:
                            current_dist_seen = d

                        if int(mid) != current_target_id and new_id is None:
                            new_id = int(mid)

                if current_dist_seen is not None:
                    last_known_dist = current_dist_seen

            if display_bgr is not None:
                dist_str = f"{last_known_dist:.2f}m"
                cv2.putText(display_bgr,
                            f"ARC {direction.upper()} from ID {current_target_id} "
                            f"({elapsed:.1f}s / {ARC_TIMEOUT:.0f}s)",
                            (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (0, 165, 255), 2)
                cv2.putText(display_bgr,
                            f"Dist:{dist_str}  Looking for new fiducial...",
                            (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0, 200, 255), 2)
                cv2.imshow("Tello ArUco Tracker", display_bgr)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("[ARC] Q pressed - aborting")
                drone.send_rc_control(0, 0, 0, 0)
                return None

            if new_id is not None:
                print(f"[ARC] Found new fiducial ID {new_id} - centering")
                break

        time.sleep(0.03)

    # Phase 2: center on new fiducial
    drone.send_rc_control(0, 0, 0, 0)
    time.sleep(0.2)

    center_start = time.time()
    in_tolerance_since = None

    while True:
        elapsed = time.time() - center_start
        if elapsed > ARC_CENTERING_TIMEOUT:
            print("[ARC] Centering timeout - handing off anyway")
            drone.send_rc_control(0, 0, 0, 0)
            return new_id

        frame = frame_reader.frame
        if frame is None or frame.shape[1] < 600:
            time.sleep(0.02)
            continue

        display_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        gray = cv2.cvtColor(display_bgr, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)

        found_new = False
        yaw_err_px = 0
        dist_err = 0.0

        if ids is not None:
            aruco.drawDetectedMarkers(display_bgr, corners, ids)
            for corner, mid in zip(corners, ids.flatten()):
                if int(mid) != new_id:
                    continue
                current_obj_points = OBJ_POINTS_SHOULDER if mid in (1, 2) else OBJ_POINTS_DEFAULT
                success, rvec, tvec = cv2.solvePnP(
                    current_obj_points, corner.reshape(4, 2),
                    camera_matrix, dist_coeffs
                )
                if not success:
                    continue
                found_new = True
                x, y, z = tvec[0, 0], tvec[1, 0], tvec[2, 0]
                dist = np.sqrt(x**2 + y**2 + z**2)
                dist_err = dist - TARGET_DIST

                fh, fw = display_bgr.shape[:2]
                marker_cx = int(corner[0, :, 0].mean())
                yaw_err_px = marker_cx - (fw // 2)
                break

        if not found_new:
            drone.send_rc_control(0, 0, 0, yaw_sign * 15)
            in_tolerance_since = None
            cv2.putText(display_bgr,
                        f"CENTERING ID {new_id} - lost, searching",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 100, 255), 2)
            cv2.imshow("Tello ArUco Tracker", display_bgr)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                drone.send_rc_control(0, 0, 0, 0)
                return None
            time.sleep(0.05)
            continue

        yaw_cmd = 0
        fb_cmd = 0

        if abs(yaw_err_px) > YAW_DEAD_ZONE:
            yaw_cmd = int(clamp(
                YAW_SPEED * (yaw_err_px / (display_bgr.shape[1] / 2)),
                -100, 100
            ))

        abs_d = abs(dist_err)
        fb_speed = fb_speed_from_error(abs_d)
        if fb_speed > 0:
            sign = 1 if dist_err > 0 else -1
            fb_cmd = int(clamp(sign * fb_speed, -FB_MAX, FB_MAX))

        drone.send_rc_control(0, fb_cmd, 0, yaw_cmd)

        in_tol = (abs(yaw_err_px) <= ARC_YAW_TOL and abs_d <= ARC_DIST_TOL)
        if in_tol:
            if in_tolerance_since is None:
                in_tolerance_since = time.time()
            elif time.time() - in_tolerance_since >= ARC_CENTER_HOLD:
                print(f"[ARC] Centered on ID {new_id} - handing off")
                drone.send_rc_control(0, 0, 0, 0)
                return new_id
        else:
            in_tolerance_since = None

        cv2.putText(display_bgr,
                    f"CENTERING ID {new_id}  yaw_err:{yaw_err_px:+d}px  "
                    f"dist_err:{dist_err:+.2f}m",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (0, 200, 100) if in_tol else (0, 165, 255), 2)
        cv2.imshow("Tello ArUco Tracker", display_bgr)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            drone.send_rc_control(0, 0, 0, 0)
            return None

        time.sleep(0.03)

def calibrate_glove_thresholds(glove_thread):
    global FINGER_UP_THRESHOLD
    print("\n" + "="*50)
    print("  GLOVE CALIBRATION")
    print("="*50)
    
    # Wait until we get a clear "released" state first (-1.0 on all fingers)
    # The default state in the class is 0.0, so this wait handles initialization
    if glove_thread.last_fingers[0] >= -0.5:
        print("Please release the button on the glove to begin...")
        while glove_thread.last_fingers[0] >= -0.5:
            time.sleep(0.05)

    print("Keep your hand flat (fingers unbent). Press the button on the glove to record.")
    while glove_thread.last_fingers[0] < -0.5:
        time.sleep(0.05)
    
    # Short delay to let the reading stabilize after the button press
    time.sleep(0.1)
    straight_voltages = glove_thread.last_fingers
    print(f"Recorded straight: {straight_voltages}")

    # Wait for the user to release the button
    while glove_thread.last_fingers[0] >= -0.5:
        time.sleep(0.05)

    print("\nBend all your fingers. Press the button on the glove to record.")
    while glove_thread.last_fingers[0] < -0.5:
        time.sleep(0.05)
        
    # Short delay to stabilize
    time.sleep(0.1)
    bent_voltages = glove_thread.last_fingers
    print(f"Recorded bent: {bent_voltages}")
    
    # Wait for release to prevent accidental triggering after calibration
    while glove_thread.last_fingers[0] >= -0.5:
        time.sleep(0.05)

    for i in range(4):
        FINGER_UP_THRESHOLD[i] = (straight_voltages[i] + bent_voltages[i]) / 2.0
    
    print("\nNew Thresholds (FINGER_UP_THRESHOLD):")
    print(f"  [0] pinky:  {FINGER_UP_THRESHOLD[0]:.3f}")
    print(f"  [1] ring:   {FINGER_UP_THRESHOLD[1]:.3f}")
    print(f"  [2] middle: {FINGER_UP_THRESHOLD[2]:.3f}")
    print(f"  [3] index:  {FINGER_UP_THRESHOLD[3]:.3f}")
    print("="*50 + "\n")


# =====================================================================
#  Main
# =====================================================================
drone = Tello()
drone.connect()
print(f"Battery: {drone.get_battery()}%")

if drone.get_battery() < 15:
    print("Battery too low to fly. Exiting.")
    exit()

drone.streamon()
time.sleep(2)

frame_reader = drone.get_frame_read()

# Start detector thread
det_thread = DetectorThread(frame_reader)
det_thread.start()

# Start video writer thread
timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
video_path = os.path.join(SCRIPT_DIR, f"tello_flight_{timestamp}{VIDEO_EXT}")
video_thread = VideoWriterThread(frame_reader, video_path)
video_thread.start()

# Start glove thread
gesture_queue = queue.Queue(maxsize=8)
glove_stop = threading.Event()
glove_thread = None
if GLOVE_ENABLED:
    glove_thread = GloveThread(gesture_queue, glove_stop)
    glove_thread.start()
    
    if CALIBRATE_THRESHOLDS:
        print("[GLOVE] Waiting for glove to connect for calibration...")
        while not glove_thread.connected:
            time.sleep(0.1)
        time.sleep(0.5) # Give it a moment to receive initial packets
        calibrate_glove_thresholds(glove_thread)
else:
    print("[GLOVE] Disabled in config - keyboard only")

if GROUND_TEST:
    print("=" * 50)
    print("  GROUND TEST MODE - DRONE WILL NOT FLY")
    print("=" * 50)
else:
    print("Taking off...")
    drone.takeoff()
    time.sleep(3)

dist_history = []
lr_history = []
vert_history = []

last_target_time = time.time()
LOST_TIMEOUT = 1.0

last_rc = (0, 0, 0, 0)
last_rc_send = 0.0


def send_rc_throttled(lr, fb, ud, yaw):
    global last_rc, last_rc_send
    if GROUND_TEST:
        return
    now = time.time()
    new_rc = (lr, fb, ud, yaw)
    if new_rc != last_rc or (now - last_rc_send) > RC_HEARTBEAT:
        if (now - last_rc_send) >= RC_MIN_INTERVAL:
            drone.send_rc_control(lr, fb, ud, yaw)
            last_rc = new_rc
            last_rc_send = now


def reset_rc_state():
    global last_rc, last_rc_send
    last_rc = (0, 0, 0, 0)
    last_rc_send = time.time()


def handle_arc(direction):
    """Run an arc maneuver and switch tracking target on success."""
    global TARGET_ID, last_target_time
    if GROUND_TEST:
        print(f"[GROUND TEST] Arc {direction} skipped")
        return

    drone.send_rc_control(0, 0, 0, 0)
    reset_rc_state()
    time.sleep(0.2)

    new_id = arc_to_next_fiducial(drone, frame_reader, TARGET_ID, direction)

    drone.send_rc_control(0, 0, 0, 0)
    reset_rc_state()

    if new_id is not None:
        TARGET_ID = new_id
        clear_histories(dist_history, lr_history, vert_history)
        print(f"[ARC] Now tracking ID {TARGET_ID}")
    else:
        print(f"[ARC] No handoff - resuming ID {TARGET_ID}")

    last_target_time = time.time()


quit_requested = False

try:
    while not quit_requested:
        # ---- Drain any pending gesture events from the glove ----
        try:
            while True:
                action = gesture_queue.get_nowait()
                if action == 'quit':
                    print("[GLOVE] Quit gesture")
                    quit_requested = True
                    break
                elif action == 'arc_left':
                    handle_arc('left')
                elif action == 'arc_right':
                    handle_arc('right')
                elif action == 'further':
                    if TARGET_DIST_INDEX < len(TARGET_DISTS) - 1:
                        TARGET_DIST_INDEX += 1
                        TARGET_DIST = TARGET_DISTS[TARGET_DIST_INDEX]
                        print(f"Target distance increased to {TARGET_DIST}m")
                elif action == 'closer':
                    if TARGET_DIST_INDEX > 0:
                        TARGET_DIST_INDEX -= 1
                        TARGET_DIST = TARGET_DISTS[TARGET_DIST_INDEX]
                        print(f"Target distance decreased to {TARGET_DIST}m")
                elif action == 'up':
                    print("[GLOVE] Moving up ~3 feet")
                    if not GROUND_TEST:
                        drone.send_rc_control(0, 0, 0, 0)
                        reset_rc_state()
                        time.sleep(0.3)
                        try:
                            drone.move_up(91)  # 3 ft = 91.44 cm
                        except Exception as e:
                            print(f"Move up failed: {e}")
                elif action == 'down':
                    print("[GLOVE] Moving down ~3 feet")
                    if not GROUND_TEST:
                        drone.send_rc_control(0, 0, 0, 0)
                        reset_rc_state()
                        time.sleep(0.3)
                        try:
                            drone.move_down(91)
                        except Exception as e:
                            print(f"Move down failed: {e}")
        except queue.Empty:
            pass

        if quit_requested:
            break

        # ---- Tracking ----
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

        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)

            for corner, marker_id in zip(corners, ids.flatten()):
                current_obj_points = OBJ_POINTS_SHOULDER if marker_id in (1, 2) else OBJ_POINTS_DEFAULT
                success, rvec, tvec = cv2.solvePnP(
                    current_obj_points, corner.reshape(4, 2),
                    camera_matrix, dist_coeffs
                )
                if not success:
                    continue

                x, y, z = tvec[0, 0], tvec[1, 0], tvec[2, 0]
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs,
                                  rvec, tvec, 0.05)

                label = f"ID {marker_id}: x={x:.2f} y={y:.2f} z={z:.2f}m"
                pos = tuple(corner[0][0].astype(int))
                cv2.putText(frame, label, (pos[0], pos[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                if marker_id == TARGET_ID:
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
                    if abs(error_x) > YAW_DEAD_ZONE:
                        yaw_cmd = int(clamp(
                            YAW_SPEED * (error_x / (frame_w / 2)),
                            -100, 100
                        ))

                    raw_dist = np.sqrt(x**2 + y**2 + z**2)
                    dist_history.append(raw_dist)
                    if len(dist_history) > SMOOTH_WINDOW:
                        dist_history.pop(0)
                    current_dist = np.median(dist_history)

                    dist_error = current_dist - TARGET_DIST
                    abs_err = abs(dist_error)

                    fb_speed = fb_speed_from_error(abs_err)
                    if fb_speed > 0:
                        sign = 1 if dist_error > 0 else -1
                        fb_cmd = int(clamp(sign * fb_speed, -FB_MAX, FB_MAX))

                    urgency = urgency_from_error(abs_err)

                    lr_history.append(x)
                    if len(lr_history) > SMOOTH_WINDOW:
                        lr_history.pop(0)
                    smooth_x = np.median(lr_history)

                    lr_error = (smooth_x * (camera_matrix[0, 0] / current_dist)
                                if current_dist else 0)
                    if abs(lr_error) > LR_DEAD_ZONE:
                        lr_cmd = int(clamp(
                            LR_SPEED * urgency * (lr_error / (frame_w / 2)),
                            -LR_MAX, LR_MAX
                        ))

                    error_y = marker_cy - center_y
                    vert_history.append(error_y)
                    if len(vert_history) > SMOOTH_WINDOW:
                        vert_history.pop(0)
                    smooth_vert = np.median(vert_history)

                    if abs(smooth_vert) > VERT_DEAD_ZONE:
                        ud_cmd = int(clamp(
                            -VERT_SPEED * urgency * (smooth_vert / (frame_h / 2)),
                            -VERT_MAX, VERT_MAX
                        ))

        if not target_found and (time.time() - last_target_time) > LOST_TIMEOUT:
            yaw_cmd = 0
            fb_cmd = 0
            lr_cmd = 0

        # ---- Keyboard fallback ----
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            quit_requested = True
            break
        elif key == ord('e'):
            ud_cmd = MANUAL_UD_SPEED
        elif key == ord('d'):
            ud_cmd = -MANUAL_UD_SPEED
        elif key == ord('f'):
            if GROUND_TEST:
                print("[GROUND TEST] Flip skipped")
            else:
                drone.send_rc_control(0, 0, 0, 0)
                reset_rc_state()
                time.sleep(0.3)
                try:
                    drone.flip("f")
                    time.sleep(1.5)
                except Exception as e:
                    print(f"Flip failed: {e}")
            continue
        elif key in (ord('a'), ord('s')):
            direction = 'left' if key == ord('a') else 'right'
            handle_arc(direction)
            continue
        elif key in [ord('0'), ord('1'), ord('2'), ord('3')]:
            new_target = key - ord('0')
            if new_target != TARGET_ID:
                TARGET_ID = new_target
                clear_histories(dist_history, lr_history, vert_history)
                print(f"Switched to target ID: {TARGET_ID}")

        # ---- HUD ----
        dist_str = f"{current_dist:.2f}m" if current_dist else "N/A"
        if current_dist is not None:
            _err = abs(current_dist - TARGET_DIST)
            _fb_tier = fb_speed_from_error(_err)
            _urgency = urgency_from_error(_err)
            tier_str = f"  TIER:{_fb_tier} URG:{_urgency:.1f}x"
        else:
            tier_str = ""

        line1 = (f"TARGET:{TARGET_ID}  YAW:{yaw_cmd:+d} FB:{fb_cmd:+d} "
                 f"LR:{lr_cmd:+d} UD:{ud_cmd:+d}")
        line2 = f"DIST:{dist_str} TARGET:{TARGET_DIST:.2f}m{tier_str}"
        lost_str = "" if target_found else " [TARGET LOST]"

        # Glove status line
        if GLOVE_ENABLED and glove_thread is not None:
            if glove_thread.connected:
                gf = glove_thread.last_fingers
                line3 = (f"GLOVE: {gf[0]:.2f},{gf[1]:.2f},{gf[2]:.2f},"
                         f"{gf[3]:.2f}V  pat:{glove_thread.last_pattern}")
            else:
                line3 = "GLOVE: disconnected"
        else:
            line3 = "GLOVE: disabled"

        cv2.putText(frame, line1 + lost_str, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
        cv2.putText(frame, line2, (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 2)
        cv2.putText(frame, line3, (10, 75),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 255, 200), 2)
        cv2.putText(frame, "REC", (frame_w - 60, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.circle(frame, (frame_w - 75, 24), 6, (0, 0, 255), -1)
        cv2.putText(frame,
                    "Glove: 1000=ArcL  1100=ArcR  1110=Quit  | Kbd: A/S/Q",
                    (10, frame_h - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)

        send_rc_throttled(lr_cmd, fb_cmd, ud_cmd, yaw_cmd)
        cv2.imshow("Tello ArUco Tracker", frame)

finally:
    print("Landing...")
    try:
        if not GROUND_TEST:
            drone.send_rc_control(0, 0, 0, 0)
            time.sleep(0.5)
            drone.land()
        else:
            print("[GROUND TEST] Skipping land")
    except Exception as e:
        print(f"Land error: {e}")

    print("Stopping glove thread...")
    glove_stop.set()
    if glove_thread is not None:
        glove_thread.join(timeout=2.0)

    print("Stopping detector thread...")
    det_thread.stop()
    det_thread.join(timeout=2.0)

    print("Stopping video writer...")
    video_thread.stop()
    video_thread.join(timeout=3.0)

    try:
        drone.streamoff()
    except Exception as e:
        print(f"Streamoff error: {e}")

    cv2.destroyAllWindows()
    print("Done.")