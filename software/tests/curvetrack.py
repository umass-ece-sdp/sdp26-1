'''
Tello ArUco tracker: takes off, holds position relative to target marker
on all axes — yaw, distance, lateral, and vertical.
Press 0-3 to switch target marker, E/D for up/down, F to flip, Q to quit.
Shift+0-3: orbit around operator until that fiducial is found.
'''

from djitellopy import Tello
import cv2
import cv2.aruco as aruco
import numpy as np
import time
import math

# --- Config ---
MARKER_SIZE = 0.213
ARUCO_DICT = aruco.DICT_4X4_50
TARGET_ID = 0
VALID_TARGETS = [0, 1, 2, 3]

# --- Control tuning (BOOSTED) ---
YAW_SPEED = 50
YAW_DEAD_ZONE = 25

TARGET_DIST = 3.048
DIST_DEAD_ZONE = 0.08
FB_SPEED = 60
FB_MAX = 80

LR_SPEED = 50
LR_DEAD_ZONE = 20
LR_MAX = 60

VERT_SPEED = 55
VERT_DEAD_ZONE = 20
VERT_MAX = 70

MANUAL_UD_SPEED = 50

SMOOTH_WINDOW = 3

# --- Orbit config ---
ORBIT_YAW_SPEED = 35        # Yaw rate during orbit (deg/s-ish, RC units)
ORBIT_LATERAL_SPEED = 45      # Lateral strafe to maintain arc
ORBIT_CHECK_INTERVAL = 0.5  # Seconds between marker checks during orbit
ORBIT_TIMEOUT = 30.0        # Max seconds to orbit before giving up

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


def clamp(val, lo, hi):
    return max(lo, min(hi, val))


def clear_histories():
    """Reset smoothing buffers when switching targets."""
    dist_history.clear()
    lr_history.clear()
    vert_history.clear()


def check_for_marker(frame_reader, target_id):
    """Grab a frame and check if the target marker is visible."""
    frame = frame_reader.frame
    if frame is None:
        return False, None
    frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is not None:
        aruco.drawDetectedMarkers(frame_bgr, corners, ids)
        for corner, mid in zip(corners, ids.flatten()):
            if mid == target_id:
                return True, frame_bgr
    return False, frame_bgr


def orbit_until_found(drone, frame_reader, target_id):
    """
    Orbit around the operator using simultaneous yaw + lateral strafe.
    
    Lateral speed is dynamically adjusted based on the distance to ANY
    visible marker — this keeps the orbit radius close to TARGET_DIST.
    If no marker is visible, falls back to a speed derived from the
    last known distance.
    """
    print(f"[ORBIT] Searching for ID {target_id} — orbiting clockwise")

    drone.send_rc_control(0, 0, 0, 0)
    time.sleep(0.3)

    start_time = time.time()
    last_check = time.time()
    last_known_dist = TARGET_DIST  # metres, start with desired distance

    while True:
        elapsed = time.time() - start_time

        if elapsed > ORBIT_TIMEOUT:
            print(f"[ORBIT] Timeout after {ORBIT_TIMEOUT}s — ID {target_id} not found")
            drone.send_rc_control(0, 0, 0, 0)
            return False

        # Check for markers and measure distance every interval
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

                        # Use distance from ANY visible marker to gauge radius
                        if current_dist is None or d < current_dist:
                            current_dist = d

                        if mid == target_id:
                            found = True

                if current_dist is not None:
                    last_known_dist = current_dist

            # Compute lateral speed to maintain TARGET_DIST orbit radius
            # If too close, strafe faster to widen the arc
            # If too far, strafe slower to tighten the arc
            ratio = TARGET_DIST / max(last_known_dist, 0.3)
            adjusted_lateral = int(clamp(
                ORBIT_LATERAL_SPEED * ratio,
                15, 80
            ))

            # Update display
            if frame_bgr is not None:
                h, w = frame_bgr.shape[:2]
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

        # Orbit: yaw right + strafe left = pivot around the camera face
        # Lateral speed is dynamically scaled to hold the target radius
        ratio = TARGET_DIST / max(last_known_dist, 0.3)
        lr = int(clamp(ORBIT_LATERAL_SPEED * ratio, 15, 80))
        drone.send_rc_control(-lr, 0, 0, ORBIT_YAW_SPEED)

        time.sleep(0.05)


# --- Connect and takeoff ---
drone = Tello()
drone.connect()
print(f"Battery: {drone.get_battery()}%")

if drone.get_battery() < 15:
    print("Battery too low to fly. Exiting.")
    exit()

drone.streamon()
time.sleep(2)

frame_reader = drone.get_frame_read()

print("Taking off...")
drone.takeoff()
time.sleep(3)

dist_history = []
lr_history = []
vert_history = []

last_target_time = time.time()
LOST_TIMEOUT = 1.0

try:
    while True:
        frame = frame_reader.frame
        if frame is None:
            continue

        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)

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
                    if abs(dist_error) > DIST_DEAD_ZONE:
                        fb_cmd = int(clamp(
                            FB_SPEED * (dist_error / TARGET_DIST),
                            -FB_MAX, FB_MAX
                        ))

                    lr_history.append(x)
                    if len(lr_history) > SMOOTH_WINDOW:
                        lr_history.pop(0)
                    smooth_x = np.median(lr_history)

                    lr_error = smooth_x * (camera_matrix[0, 0] / current_dist) if current_dist else 0
                    if abs(lr_error) > LR_DEAD_ZONE:
                        lr_cmd = int(clamp(
                            LR_SPEED * (lr_error / (frame_w / 2)),
                            -LR_MAX, LR_MAX
                        ))

                    error_y = marker_cy - center_y
                    vert_history.append(error_y)
                    if len(vert_history) > SMOOTH_WINDOW:
                        vert_history.pop(0)
                    smooth_vert = np.median(vert_history)

                    if abs(smooth_vert) > VERT_DEAD_ZONE:
                        ud_cmd = int(clamp(
                            -VERT_SPEED * (smooth_vert / (frame_h / 2)),
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
            drone.send_rc_control(0, 0, 0, 0)
            time.sleep(0.3)
            try:
                drone.flip("f")
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

            drone.send_rc_control(0, 0, 0, 0)
            time.sleep(0.3)

            found = orbit_until_found(drone, frame_reader, orbit_target)

            if found:
                TARGET_ID = orbit_target
                clear_histories()
                print(f"[ORBIT] Now tracking ID {TARGET_ID}")
            else:
                print(f"[ORBIT] ID {orbit_target} not found. Resuming ID {TARGET_ID}")

            last_target_time = time.time()
            continue

        # Normal 0-3: instant target switch (no orbiting)
        elif key in [ord('0'), ord('1'), ord('2'), ord('3')]:
            new_target = key - ord('0')
            if new_target != TARGET_ID:
                TARGET_ID = new_target
                clear_histories()
                print(f"Switched to target ID: {TARGET_ID}")

        # --- Status display ---
        dist_str = f"{current_dist:.2f}m" if current_dist else "N/A"
        line1 = f"TARGET:{TARGET_ID}  YAW:{yaw_cmd:+d} FB:{fb_cmd:+d} LR:{lr_cmd:+d} UD:{ud_cmd:+d}"
        line2 = f"DIST:{dist_str} TARGET:{TARGET_DIST:.2f}m"
        lost_str = "" if target_found else " [TARGET LOST]"

        cv2.putText(frame, line1 + lost_str, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
        cv2.putText(frame, line2, (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 2)
        cv2.putText(frame, "0-3=Target  Shift+0-3=Orbit  E/D=Up/Down  F=Flip  Q=Quit",
                    (10, frame_h - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

        drone.send_rc_control(lr_cmd, fb_cmd, ud_cmd, yaw_cmd)

        cv2.imshow("Tello ArUco Tracker", frame)

finally:
    print("Landing...")
    drone.send_rc_control(0, 0, 0, 0)
    time.sleep(0.5)
    drone.land()
    drone.streamoff()
    cv2.destroyAllWindows()