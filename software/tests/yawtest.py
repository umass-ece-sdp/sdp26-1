'''
Tello ArUco tracker: takes off, detects fiducial markers,
and yaws to keep the target marker centered in the camera view.
Press E to go up, D to go down, Q to quit and land.
'''

from djitellopy import Tello
import cv2
import cv2.aruco as aruco
import numpy as np
import time

# --- Config ---
MARKER_SIZE = 0.105  # marker size in meters
ARUCO_DICT = aruco.DICT_4X4_50
TARGET_ID = 0

# Control tuning
YAW_SPEED = 30        # base yaw rotation speed (0-100)
DEAD_ZONE = 40        # pixels from center before we rotate
UD_SPEED = 30         # up/down speed when pressing E/D
UD_DURATION = 0.5     # seconds to hold the up/down command

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

try:
    while True:
        frame = frame_reader.frame
        if frame is None:
            continue

        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)

        yaw_cmd = 0
        ud_cmd = 0
        frame_h, frame_w = frame.shape[:2]
        center_x = frame_w // 2

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

                # --- Track the target marker ---
                if marker_id == TARGET_ID:
                    marker_center_x = int(corner[0, :, 0].mean())
                    marker_center_y = int(corner[0, :, 1].mean())

                    cv2.circle(frame, (marker_center_x, marker_center_y), 8, (0, 0, 255), 2)
                    cv2.line(frame, (center_x, frame_h // 2 - 15),
                             (center_x, frame_h // 2 + 15), (255, 0, 0), 1)

                    error_x = marker_center_x - center_x

                    if abs(error_x) > DEAD_ZONE:
                        yaw_cmd = int(clamp(
                            YAW_SPEED * (error_x / (frame_w / 2)),
                            -100, 100
                        ))

        # --- Handle keyboard input ---
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break
        elif key == ord('e'):
            ud_cmd = UD_SPEED
        elif key == ord('d'):
            ud_cmd = -UD_SPEED

        # Show status on frame
        status = f"YAW: {yaw_cmd}  UD: {ud_cmd}"
        cv2.putText(frame, status, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.putText(frame, "E=Up  D=Down  Q=Quit", (10, frame_h - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # Send RC command: (left/right, fwd/back, up/down, yaw)
        drone.send_rc_control(0, 0, ud_cmd, yaw_cmd)

        cv2.imshow("Tello ArUco Tracker", frame)

finally:
    print("Landing...")
    drone.send_rc_control(0, 0, 0, 0)
    time.sleep(0.5)
    drone.land()
    drone.streamoff()
    cv2.destroyAllWindows()