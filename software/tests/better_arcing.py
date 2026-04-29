'''
Tello ArUco tracker: takes off, holds position relative to target marker
on all axes — yaw, distance, lateral, and vertical.
Press 0-3 to switch target marker, E/D for up/down, F to flip, Q to quit.
Shift+0-3: orbit around operator until that fiducial is found.

Threaded version:
  - Detector runs in its own thread (non-blocking main loop)
  - Raw video stream is saved to a timestamped MP4 via a writer thread
'''

from djitellopy import Tello
import cv2
import cv2.aruco as aruco
import numpy as np
import time
import math
import os
import threading
import queue
from datetime import datetime

# --- Debug ---
GROUND_TEST = False  # Set False for real flight. Skips takeoff/land/RC sends.

# --- Config ---
MARKER_SIZE = 0.213
ARUCO_DICT = aruco.DICT_4X4_50
TARGET_ID = 0
VALID_TARGETS = [0, 1, 2, 3]

# --- Control tuning ---
YAW_SPEED = 50
YAW_DEAD_ZONE = 25

TARGET_DIST = 3.048
DIST_DEAD_ZONE = 0.08

# FB: threshold-based speed tiers keyed on distance error magnitude.
# Each tuple is (min_error_meters, fb_speed). First match wins.
# Below DIST_DEAD_ZONE → fb_cmd = 0 (hover in zone).
FB_TIERS = [
    (2.0, 100),   # > 2m off   → full speed
    (1.0,  75),   # > 1m off   → cruise
    (0.4,  45),   # > 0.4m off → approach
    (0.08, 20),   # > deadzone → creep
]
FB_MAX = 100

LR_SPEED = 70
LR_DEAD_ZONE = 20
LR_MAX = 100

VERT_SPEED = 75
VERT_DEAD_ZONE = 20
VERT_MAX = 100

# Urgency: LR and vertical gains get multiplied by a tier-based boost
# when distance error is large. Same structure as FB_TIERS.
# Below smallest threshold → multiplier = 1.0.
URGENCY_TIERS = [
    (2.0, 1.8),   # > 2m off   → 80% boost
    (1.0, 1.4),   # > 1m off   → 40% boost
    (0.4, 1.1),   # > 0.4m off → 10% boost
]

MANUAL_UD_SPEED = 50

SMOOTH_WINDOW = 3

# --- Orbit config ---
ORBIT_YAW_SPEED = 35
ORBIT_LATERAL_SPEED = 45
ORBIT_CHECK_INTERVAL = 0.5
ORBIT_TIMEOUT = 30.0

# --- Arc-to-next-fiducial config ---
ARC_YAW_SPEED = 35              # baseline yaw at TARGET_DIST (3.05m)
ARC_LATERAL_SPEED = 45          # baseline lateral at TARGET_DIST (3.05m)
ARC_CHECK_INTERVAL = 0.15       # check for new fiducials more often than orbit
ARC_TIMEOUT = 20.0              # max seconds to arc before giving up
ARC_CENTERING_TIMEOUT = 4.0     # max seconds to spend centering on new fiducial
ARC_CENTER_HOLD = 0.5           # must be centered for this long to hand off
ARC_YAW_TOL = YAW_DEAD_ZONE * 2 # pixels off-center yaw tolerance
ARC_DIST_TOL = DIST_DEAD_ZONE * 2  # meters distance tolerance

# Arc speed scales with TARGET_DIST so a wider orbit covers the same
# angular sweep per second as a tight one.
#   lateral scales linearly with radius (arc length = r * theta)
#   yaw scales gently — keeps drone pointed at operator without overshooting
ARC_REFERENCE_DIST = 3.048      # distance at which the baselines apply
ARC_YAW_EXPONENT = 0.5          # 0=no scaling, 1=linear; sqrt feels best
ARC_LATERAL_MIN = 15
ARC_LATERAL_MAX = 90
ARC_YAW_MIN = 18
ARC_YAW_MAX = 60
ARC_STRAFE_ONLY_DURATION = 0.3  # seconds of pure strafe before yaw kicks in

# --- RC throttling (to reduce WiFi contention) ---
RC_MIN_INTERVAL = 0.08   # ~12 Hz max send rate
RC_HEARTBEAT = 0.25      # resend identical command at least this often

# --- Video recording ---
VIDEO_FPS = 30
VIDEO_QUEUE_SIZE = 60    # ~2 seconds of buffer
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# --- ArUco Setup ---
aruco_dict = aruco.getPredefinedDictionary(ARUCO_DICT)
detector_params = aruco.DetectorParameters()
# Speed tuning: subpixel refinement is overkill for 3m tracking
detector_params.cornerRefinementMethod = aruco.CORNER_REFINE_NONE
detector = aruco.ArucoDetector(aruco_dict, detector_params)

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


def clamp(val, lo, hi):
    return max(lo, min(hi, val))


def fb_speed_from_error(abs_err):
    """Look up FB speed magnitude from the tier table. 0 if inside deadzone."""
    for threshold, speed in FB_TIERS:
        if abs_err > threshold:
            return speed
    return 0


def urgency_from_error(abs_err):
    """Look up LR/vertical urgency multiplier from the tier table."""
    for threshold, mult in URGENCY_TIERS:
        if abs_err > threshold:
            return mult
    return 1.0


def arc_speeds_for_distance(target_dist):
    """
    Scale arc lateral & yaw speeds so the angular sweep rate stays roughly
    constant regardless of how big the orbit radius is.

    Returns (lateral, yaw) ints, both clamped to safe ranges.
    """
    ratio = max(target_dist, 0.3) / ARC_REFERENCE_DIST
    lateral = int(clamp(ARC_LATERAL_SPEED * ratio,
                        ARC_LATERAL_MIN, ARC_LATERAL_MAX))
    yaw = int(clamp(ARC_YAW_SPEED * (ratio ** ARC_YAW_EXPONENT),
                    ARC_YAW_MIN, ARC_YAW_MAX))
    return lateral, yaw


# =====================================================================
#  Detector thread — runs ArUco detection on the latest frame in a loop
# =====================================================================
class DetectorThread(threading.Thread):
    """
    Continuously detects ArUco markers on the latest frame from the drone.
    Main loop reads `.result` (a snapshot dict) under `.lock`.
    Always works on the freshest frame — skips stale ones.
    """
    def __init__(self, frame_reader):
        super().__init__(daemon=True)
        self.frame_reader = frame_reader
        self.lock = threading.Lock()
        self.running = True
        # Shared result (read by main thread)
        self.result = {
            'corners': None,
            'ids': None,
            'frame': None,          # BGR frame detection ran on
            'timestamp': 0.0,
        }

    def run(self):
        # Run detection as fast as possible. Tiny sleep yields the GIL
        # so the video writer and main thread aren't starved.
        # Skip placeholder frames (< 600px wide) that appear before the
        # H.264 decoder has real data from the drone.
        while self.running:
            frame = self.frame_reader.frame
            if frame is None:
                time.sleep(0.005)
                continue

            if frame.shape[1] < 600:
                # Placeholder frame, stream not yet live
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

            # Yield briefly — no point running faster than frames arrive
            time.sleep(0.01)

    def snapshot(self):
        with self.lock:
            return dict(self.result)

    def stop(self):
        self.running = False


# =====================================================================
#  Video writer thread — pulls raw frames and writes to disk
# =====================================================================
class VideoWriterThread(threading.Thread):
    """
    Saves the raw RGB stream (converted to BGR) to an MJPG/AVI file.
    Saves the raw RGB stream (converted to BGR) to an MP4 at RECORD_FPS.
    Uses a simple fixed-rate timer like the old working script — no
    object-id de-duping, since djitellopy reuses the same numpy buffer
    and id() comparison would skip nearly every frame.
    """
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
        expected_shape = None  # (h, w) locked in once writer opens
        stable_count = 0       # consecutive frames at the same real resolution
        STABLE_REQUIRED = 5    # frames needed before we trust a resolution
        MIN_REAL_W = 600       # anything smaller is a placeholder (Tello is 960x720)

        while self.running:
            # Wait for the next tick
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

            # --- Writer not yet open: wait for a real, stable-size frame ---
            if self.writer is None:
                if w < MIN_REAL_W:
                    # Still a placeholder frame from before the decoder has data
                    continue
                stable_count += 1
                if stable_count < STABLE_REQUIRED:
                    continue

                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                self.writer = cv2.VideoWriter(
                    self.filepath, fourcc, VIDEO_FPS, (w, h)
                )
                if not self.writer.isOpened():
                    print(f"[VIDEO] Failed to open writer at {self.filepath}")
                    self.running = False
                    return
                expected_shape = (h, w)
                print(f"[VIDEO] Recording {w}x{h} @ {VIDEO_FPS} FPS -> {self.filepath}")

            # --- Writer open: enforce shape match ---
            if (h, w) != expected_shape:
                # Silently drop mismatched frames — writer would reject them anyway
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
#  Helpers
# =====================================================================
def clear_histories(dist_history, lr_history, vert_history):
    dist_history.clear()
    lr_history.clear()
    vert_history.clear()


def orbit_until_found(drone, frame_reader, target_id):
    """
    Orbit around the operator using simultaneous yaw + lateral strafe.
    Uses inline detection (not the async detector) because we need
    synchronous "found yet?" answers during the search.
    """
    print(f"[ORBIT] Searching for ID {target_id} — orbiting clockwise")

    drone.send_rc_control(0, 0, 0, 0)
    time.sleep(0.3)

    start_time = time.time()
    last_check = time.time()
    last_known_dist = TARGET_DIST

    while True:
        elapsed = time.time() - start_time

        if elapsed > ORBIT_TIMEOUT:
            print(f"[ORBIT] Timeout after {ORBIT_TIMEOUT}s — ID {target_id} not found")
            drone.send_rc_control(0, 0, 0, 0)
            return False

        if time.time() - last_check >= ORBIT_CHECK_INTERVAL:
            last_check = time.time()

            found = False
            frame = frame_reader.frame
            frame_bgr = None
            current_dist = None

            if frame is not None:
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
                corners, ids, _ = detector.detectMarkers(gray)

                if ids is not None:
                    aruco.drawDetectedMarkers(frame_bgr, corners, ids)

                    for corner, mid in zip(corners, ids.flatten()):
                        success, rvec, tvec = cv2.solvePnP(
                            obj_points, corner.reshape(4, 2),
                            camera_matrix, dist_coeffs
                        )
                        if not success:
                            continue

                        x, y, z = tvec[0, 0], tvec[1, 0], tvec[2, 0]
                        d = np.sqrt(x**2 + y**2 + z**2)

                        if current_dist is None or d < current_dist:
                            current_dist = d

                        if mid == target_id:
                            found = True

                if current_dist is not None:
                    last_known_dist = current_dist

            ratio = TARGET_DIST / max(last_known_dist, 0.3)
            adjusted_lateral = int(clamp(ORBIT_LATERAL_SPEED * ratio, 15, 80))

            if frame_bgr is not None:
                dist_str = f"{last_known_dist:.2f}m" if last_known_dist else "?"
                cv2.putText(frame_bgr,
                            f"ORBITING for ID {target_id} ({elapsed:.1f}s / {ORBIT_TIMEOUT:.0f}s)",
                            (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
                cv2.putText(frame_bgr,
                            f"Dist:{dist_str}  Target:{TARGET_DIST:.2f}m  Lateral:{adjusted_lateral}",
                            (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 2)
                cv2.imshow("Tello ArUco Tracker", frame_bgr)

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

        ratio = TARGET_DIST / max(last_known_dist, 0.3)
        lr = int(clamp(ORBIT_LATERAL_SPEED * ratio, 15, 80))
        drone.send_rc_control(-lr, 0, 0, ORBIT_YAW_SPEED)

        time.sleep(0.05)


def arc_to_next_fiducial(drone, frame_reader, current_target_id, direction):
    """
    Arc around the operator in the given direction ('left' or 'right')
    until any fiducial OTHER than current_target_id is detected, then
    center on it (yaw + distance) and return its ID.

    Returns:
      new_target_id (int) if successful
      None if timed out, aborted, or centering failed
    """
    if direction not in ('left', 'right'):
        print(f"[ARC] Invalid direction: {direction}")
        return None

    # Direction: 'right' = drone moves right around operator
    #   = yaw right + strafe left
    # 'left' is the mirror.
    if direction == 'right':
        yaw_sign = 1   # yaw right
        lr_sign = -1   # strafe left
    else:
        yaw_sign = -1  # yaw left
        lr_sign = 1    # strafe right

    print(f"[ARC] Arcing {direction} from ID {current_target_id} "
          f"to find next fiducial")

    drone.send_rc_control(0, 0, 0, 0)
    time.sleep(0.3)

    # ===== Phase 1: arc until a different fiducial appears =====
    start_time = time.time()
    last_check = 0.0
    last_known_dist = TARGET_DIST
    new_id = None
    new_corner = None

    # Compute baseline arc speeds for the current TARGET_DIST.
    # These stay constant for this arc; the radius-correction below
    # only nudges them when the drone drifts toward/away from operator.
    base_lateral, base_yaw = arc_speeds_for_distance(TARGET_DIST)
    print(f"[ARC] Speeds for TARGET_DIST={TARGET_DIST:.2f}m: "
          f"lateral={base_lateral} yaw={base_yaw}")

    # Local throttle so we don't hammer WiFi inside this loop
    last_arc_rc = None
    last_arc_send = 0.0
    ARC_RC_INTERVAL = 0.08  # ~12 Hz, same as main loop throttle

    while True:
        elapsed = time.time() - start_time
        if elapsed > ARC_TIMEOUT:
            print(f"[ARC] Timeout after {ARC_TIMEOUT}s — no new fiducial")
            drone.send_rc_control(0, 0, 0, 0)
            return None

        # Drive the arc — scale lateral by current radius drift,
        # but yaw stays fixed (it's an angular rate, not linear).
        # For the first ARC_STRAFE_ONLY_DURATION seconds, yaw is held at 0
        # so the drone moves sideways before turning to face the operator.
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

        # Periodically check the camera
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
                        # Pose for distance estimate
                        success, rvec, tvec = cv2.solvePnP(
                            obj_points, corner.reshape(4, 2),
                            camera_matrix, dist_coeffs
                        )
                        if not success:
                            continue
                        x, y, z = tvec[0, 0], tvec[1, 0], tvec[2, 0]
                        d = np.sqrt(x**2 + y**2 + z**2)
                        if current_dist_seen is None or d < current_dist_seen:
                            current_dist_seen = d

                        # Found a NEW fiducial (different from current target)
                        if int(mid) != current_target_id and new_id is None:
                            new_id = int(mid)
                            new_corner = corner

                if current_dist_seen is not None:
                    last_known_dist = current_dist_seen

            # HUD update during arc
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
                print("[ARC] Q pressed — aborting")
                drone.send_rc_control(0, 0, 0, 0)
                return None

            if new_id is not None:
                print(f"[ARC] Found new fiducial ID {new_id} — centering")
                break

        time.sleep(0.03)

    # ===== Phase 2: center on the new fiducial =====
    drone.send_rc_control(0, 0, 0, 0)
    time.sleep(0.2)

    center_start = time.time()
    in_tolerance_since = None

    while True:
        elapsed = time.time() - center_start
        if elapsed > ARC_CENTERING_TIMEOUT:
            print(f"[ARC] Centering timeout — handing off anyway")
            drone.send_rc_control(0, 0, 0, 0)
            return new_id  # Hand off to normal tracking, it'll fine-tune

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
                success, rvec, tvec = cv2.solvePnP(
                    obj_points, corner.reshape(4, 2),
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
            # Lost it — slow rotation back toward direction we came from
            drone.send_rc_control(0, 0, 0, yaw_sign * 15)
            in_tolerance_since = None
            cv2.putText(display_bgr,
                        f"CENTERING ID {new_id} — lost, searching",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 100, 255), 2)
            cv2.imshow("Tello ArUco Tracker", display_bgr)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                drone.send_rc_control(0, 0, 0, 0)
                return None
            time.sleep(0.05)
            continue

        # Build centering RC: just yaw + FB (using same tier system)
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

        # Check if we're in tolerance
        in_tol = (abs(yaw_err_px) <= ARC_YAW_TOL and abs_d <= ARC_DIST_TOL)
        if in_tol:
            if in_tolerance_since is None:
                in_tolerance_since = time.time()
            elif time.time() - in_tolerance_since >= ARC_CENTER_HOLD:
                print(f"[ARC] Centered on ID {new_id} — handing off")
                drone.send_rc_control(0, 0, 0, 0)
                return new_id
        else:
            in_tolerance_since = None

        # HUD
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
video_path = os.path.join(SCRIPT_DIR, f"tello_flight_{timestamp}.mp4")
video_thread = VideoWriterThread(frame_reader, video_path)
video_thread.start()

if GROUND_TEST:
    print("=" * 50)
    print("  GROUND TEST MODE — DRONE WILL NOT FLY")
    print("=" * 50)
    print("[GROUND TEST] Skipping takeoff — stream + detection + recording only")
else:
    print("Taking off...")
    drone.takeoff()
    time.sleep(3)

dist_history = []
lr_history = []
vert_history = []

last_target_time = time.time()
LOST_TIMEOUT = 1.0

# RC throttling state
last_rc = (0, 0, 0, 0)
last_rc_send = 0.0


def send_rc_throttled(lr, fb, ud, yaw):
    """Only send if command changed or heartbeat interval elapsed."""
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


try:
    while True:
        snap = det_thread.snapshot()
        frame = snap['frame']

        if frame is None:
            time.sleep(0.01)
            continue

        # Work on a copy for HUD drawing (don't mutate detector's frame)
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
                success, rvec, tvec = cv2.solvePnP(
                    obj_points, corner.reshape(4, 2),
                    camera_matrix, dist_coeffs
                )
                if not success:
                    continue

                x, y, z = tvec[0, 0], tvec[1, 0], tvec[2, 0]
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.05)

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

                    # FB: threshold-tiered speed, signed by error direction
                    fb_speed = fb_speed_from_error(abs_err)
                    if fb_speed > 0:
                        sign = 1 if dist_error > 0 else -1
                        fb_cmd = int(clamp(sign * fb_speed, -FB_MAX, FB_MAX))

                    # Urgency multiplier for LR and vertical — bigger when
                    # we're far from target distance
                    urgency = urgency_from_error(abs_err)

                    lr_history.append(x)
                    if len(lr_history) > SMOOTH_WINDOW:
                        lr_history.pop(0)
                    smooth_x = np.median(lr_history)

                    lr_error = smooth_x * (camera_matrix[0, 0] / current_dist) if current_dist else 0
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

        # --- Keyboard input ---
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break
        elif key == ord('e'):
            ud_cmd = MANUAL_UD_SPEED
        elif key == ord('d'):
            ud_cmd = -MANUAL_UD_SPEED
        elif key == ord('f'):
            if GROUND_TEST:
                print("[GROUND TEST] Flip skipped")
                continue
            drone.send_rc_control(0, 0, 0, 0)
            last_rc = (0, 0, 0, 0)
            last_rc_send = time.time()
            time.sleep(0.3)
            try:
                drone.flip("f")
                time.sleep(1.5)
            except Exception as e:
                print(f"Flip failed: {e}")
            continue

        elif key in (ord('a'), ord('s')):
            direction = 'left' if key == ord('a') else 'right'
            if GROUND_TEST:
                print(f"[GROUND TEST] Arc {direction} skipped")
                continue

            drone.send_rc_control(0, 0, 0, 0)
            last_rc = (0, 0, 0, 0)
            last_rc_send = time.time()
            time.sleep(0.2)

            new_id = arc_to_next_fiducial(
                drone, frame_reader, TARGET_ID, direction
            )

            # Reset throttle state — arc bypassed it via direct sends
            drone.send_rc_control(0, 0, 0, 0)
            last_rc = (0, 0, 0, 0)
            last_rc_send = time.time()

            if new_id is not None:
                TARGET_ID = new_id
                clear_histories(dist_history, lr_history, vert_history)
                print(f"[ARC] Now tracking ID {TARGET_ID}")
            else:
                print(f"[ARC] No handoff — resuming ID {TARGET_ID}")

            last_target_time = time.time()
            continue

        elif key in [ord(')'), ord('!'), ord('@'), ord('#')]:
            shift_map = {ord(')'): 0, ord('!'): 1, ord('@'): 2, ord('#'): 3}
            orbit_target = shift_map[key]
            if GROUND_TEST:
                print(f"[GROUND TEST] Orbit to ID {orbit_target} skipped — "
                      f"switching target instead")
                TARGET_ID = orbit_target
                clear_histories(dist_history, lr_history, vert_history)
                last_target_time = time.time()
                continue
            print(f"[INPUT] Shift+{orbit_target} — orbiting to find ID {orbit_target}")

            drone.send_rc_control(0, 0, 0, 0)
            last_rc = (0, 0, 0, 0)
            last_rc_send = time.time()
            time.sleep(0.3)

            found = orbit_until_found(drone, frame_reader, orbit_target)

            if found:
                TARGET_ID = orbit_target
                clear_histories(dist_history, lr_history, vert_history)
                print(f"[ORBIT] Now tracking ID {TARGET_ID}")
            else:
                print(f"[ORBIT] ID {orbit_target} not found. Resuming ID {TARGET_ID}")

            last_target_time = time.time()
            continue

        elif key in [ord('0'), ord('1'), ord('2'), ord('3')]:
            new_target = key - ord('0')
            if new_target != TARGET_ID:
                TARGET_ID = new_target
                clear_histories(dist_history, lr_history, vert_history)
                print(f"Switched to target ID: {TARGET_ID}")

        # --- Status display ---
        dist_str = f"{current_dist:.2f}m" if current_dist else "N/A"
        if current_dist is not None:
            _err = abs(current_dist - TARGET_DIST)
            _fb_tier = fb_speed_from_error(_err)
            _urgency = urgency_from_error(_err)
            tier_str = f"  TIER:{_fb_tier} URG:{_urgency:.1f}x"
        else:
            tier_str = ""
        line1 = f"TARGET:{TARGET_ID}  YAW:{yaw_cmd:+d} FB:{fb_cmd:+d} LR:{lr_cmd:+d} UD:{ud_cmd:+d}"
        line2 = f"DIST:{dist_str} TARGET:{TARGET_DIST:.2f}m{tier_str}"
        lost_str = "" if target_found else " [TARGET LOST]"

        cv2.putText(frame, line1 + lost_str, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
        cv2.putText(frame, line2, (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 2)
        cv2.putText(frame, "REC", (frame_w - 60, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.circle(frame, (frame_w - 75, 24), 6, (0, 0, 255), -1)
        cv2.putText(frame, "0-3=Target  A/S=Arc L/R  Shift+0-3=Orbit  E/D=Up/Down  F=Flip  Q=Quit",
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

    # Stop threads before releasing stream
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