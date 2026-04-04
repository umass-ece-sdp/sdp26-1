'''
Tello ArUco tracker: takes off, holds position relative to target marker
on all axes — yaw, distance, lateral, and vertical.
Press E to go up, D to go down, Q to quit and land.
'''

from djitellopy import Tello
import cv2
import cv2.aruco as aruco
import numpy as np
import time

# --- Config ---
MARKER_SIZE = 0.105
ARUCO_DICT = aruco.DICT_4X4_50
TARGET_ID = 0

# --- Control tuning ---
# Yaw (rotation)
YAW_SPEED = 35
YAW_DEAD_ZONE = 25

# Forward/back (distance hold)
TARGET_DIST = 1.524       # 5 feet in meters
DIST_DEAD_ZONE = 0.08
FB_SPEED = 40
FB_MAX = 55

# Left/right (lateral drift correction)
LR_SPEED = 35
LR_DEAD_ZONE = 20
LR_MAX = 50

# Up/down (vertical drift correction)
VERT_SPEED = 30
VERT_DEAD_ZONE = 20
VERT_MAX = 45

# Manual up/down override
MANUAL_UD_SPEED = 30

# Smoothing
SMOOTH_WINDOW = 5

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

def clamp(val, lo, hi):
    return max(lo, min(hi, val))

# Rolling buffers for smoothing
dist_history = []
lr_history = []
vert_history = []

# Track how long since we last saw the target
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

                    # Draw crosshair
                    cv2.circle(frame, (marker_cx, marker_cy), 8, (0, 0, 255), 2)
                    cv2.line(frame, (center_x - 15, center_y),
                             (center_x + 15, center_y), (255, 0, 0), 1)
                    cv2.line(frame, (center_x, center_y - 15),
                             (center_x, center_y + 15), (255, 0, 0), 1)

                    # --- 1. Yaw: keep marker horizontally centered ---
                    error_x = marker_cx - center_x
                    if abs(error_x) > YAW_DEAD_ZONE:
                        yaw_cmd = int(clamp(
                            YAW_SPEED * (error_x / (frame_w / 2)),
                            -100, 100
                        ))

                    # --- 2. Forward/back: hold target distance ---
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

                    # --- 3. Left/right: fight lateral drift ---
                    lr_error = marker_cx - center_x
                    lr_history.append(lr_error)
                    if len(lr_history) > SMOOTH_WINDOW:
                        lr_history.pop(0)
                    smooth_lr = np.median(lr_history)

                    if abs(smooth_lr) > LR_DEAD_ZONE:
                        lr_cmd = int(clamp(
                            LR_SPEED * (smooth_lr / (frame_w / 2)),
                            -LR_MAX, LR_MAX
                        ))

                    # --- 4. Up/down: keep marker vertically centered ---
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

        # If target lost for too long, stop all movement
        if not target_found and (time.time() - last_target_time) > LOST_TIMEOUT:
            yaw_cmd = 0
            fb_cmd = 0
            lr_cmd = 0

        # --- Manual keyboard override for up/down ---
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break
        elif key == ord('e'):
            ud_cmd = MANUAL_UD_SPEED
        elif key == ord('d'):
            ud_cmd = -MANUAL_UD_SPEED

        # --- Status display ---
        dist_str = f"{current_dist:.2f}m" if current_dist else "N/A"
        line1 = f"YAW:{yaw_cmd:+d} FB:{fb_cmd:+d} LR:{lr_cmd:+d} UD:{ud_cmd:+d}"
        line2 = f"DIST:{dist_str} TARGET:{TARGET_DIST:.2f}m"
        lost_str = "" if target_found else " [TARGET LOST]"

        cv2.putText(frame, line1 + lost_str, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
        cv2.putText(frame, line2, (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 2)
        cv2.putText(frame, "E=Up  D=Down  Q=Quit", (10, frame_h - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # Send RC command: (left/right, fwd/back, up/down, yaw)
        drone.send_rc_control(lr_cmd, fb_cmd, ud_cmd, yaw_cmd)

        cv2.imshow("Tello ArUco Tracker", frame)

finally:
    print("Landing...")
    drone.send_rc_control(0, 0, 0, 0)
    time.sleep(0.5)
    drone.land()
    drone.streamoff()
    cv2.destroyAllWindows()