'''
Tello ArUco tracker: takes off, holds position relative to target marker
on all axes — yaw, distance, lateral, and vertical.
Press 0-3 to switch target marker, E/D for up/down, F to flip, Q to quit.
Video is always saved (clean, no overlays) on exit.
'''

from djitellopy import Tello
import cv2
import cv2.aruco as aruco
import numpy as np
import time
from datetime import datetime

# --- Config ---
MARKER_SIZE = 0.105
ARUCO_DICT = aruco.DICT_4X4_50
TARGET_ID = 3
VALID_TARGETS = [0, 1, 2, 3]

# --- Recording config ---
RECORD_FPS = 30
RECORD_CODEC = 'mp4v'

# --- Control tuning (BOOSTED) ---
YAW_SPEED = 50
YAW_DEAD_ZONE = 25

TARGET_DIST = 1.524
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

def clear_histories():
    """Reset smoothing buffers when switching targets."""
    dist_history.clear()
    lr_history.clear()
    vert_history.clear()

dist_history = []
lr_history = []
vert_history = []

last_target_time = time.time()
LOST_TIMEOUT = 1.0

# --- Video writer (clean frames, no overlays) ---
video_writer = None
video_filename = None

try:
    while True:
        frame = frame_reader.frame
        if frame is None:
            continue

        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        # Write the clean frame before any annotations are drawn
        if video_writer is None:
            frame_h, frame_w = frame.shape[:2]
            video_filename = f"tello_{datetime.now().strftime('%Y%m%d_%H%M%S')}.mp4"
            fourcc = cv2.VideoWriter_fourcc(*RECORD_CODEC)
            video_writer = cv2.VideoWriter(video_filename, fourcc, RECORD_FPS, (frame_w, frame_h))
            print(f"[REC] Recording to {video_filename}")
        video_writer.write(frame)

        # From here on, draw overlays only on a display copy
        display = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)

        yaw_cmd = 0
        fb_cmd = 0
        lr_cmd = 0
        ud_cmd = 0
        frame_h, frame_w = display.shape[:2]
        center_x = frame_w // 2
        center_y = frame_h // 2
        current_dist = None
        target_found = False

        if ids is not None:
            aruco.drawDetectedMarkers(display, corners, ids)

            for corner, marker_id in zip(corners, ids.flatten()):
                success, rvec, tvec = cv2.solvePnP(
                    obj_points, corner.reshape(4, 2),
                    camera_matrix, dist_coeffs
                )
                if not success:
                    continue

                x, y, z = tvec[0, 0], tvec[1, 0], tvec[2, 0]
                cv2.drawFrameAxes(display, camera_matrix, dist_coeffs, rvec, tvec, 0.05)

                label = f"ID {marker_id}: x={x:.2f} y={y:.2f} z={z:.2f}m"
                pos = tuple(corner[0][0].astype(int))
                cv2.putText(display, label, (pos[0], pos[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                if marker_id == TARGET_ID:
                    target_found = True
                    last_target_time = time.time()

                    marker_cx = int(corner[0, :, 0].mean())
                    marker_cy = int(corner[0, :, 1].mean())

                    cv2.circle(display, (marker_cx, marker_cy), 8, (0, 0, 255), 2)
                    cv2.line(display, (center_x - 15, center_y),
                             (center_x + 15, center_y), (255, 0, 0), 1)
                    cv2.line(display, (center_x, center_y - 15),
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
        elif key in [ord('0'), ord('1'), ord('2'), ord('3')]:
            new_target = key - ord('0')
            if new_target != TARGET_ID:
                TARGET_ID = new_target
                clear_histories()
                print(f"Switched to target ID: {TARGET_ID}")

        # --- Status display (on display copy only) ---
        dist_str = f"{current_dist:.2f}m" if current_dist else "N/A"
        line1 = f"TARGET:{TARGET_ID}  YAW:{yaw_cmd:+d} FB:{fb_cmd:+d} LR:{lr_cmd:+d} UD:{ud_cmd:+d}"
        line2 = f"DIST:{dist_str} TARGET:{TARGET_DIST:.2f}m"
        lost_str = "" if target_found else " [TARGET LOST]"

        cv2.putText(display, line1 + lost_str, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
        cv2.putText(display, line2, (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 2)
        cv2.putText(display, "0-3=Target  E=Up  D=Down  F=Flip  Q=Quit", (10, frame_h - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # Recording indicator on display
        cv2.circle(display, (frame_w - 25, 25), 8, (0, 0, 255), -1)
        cv2.putText(display, "REC", (frame_w - 70, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        drone.send_rc_control(lr_cmd, fb_cmd, ud_cmd, yaw_cmd)

        cv2.imshow("Tello ArUco Tracker", display)

finally:
    print("Landing...")
    drone.send_rc_control(0, 0, 0, 0)
    time.sleep(0.5)
    drone.land()
    drone.streamoff()
    if video_writer is not None:
        video_writer.release()
        print(f"[REC] Video saved: {video_filename}")
    cv2.destroyAllWindows()