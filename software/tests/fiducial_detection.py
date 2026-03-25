'''
Opens a window with OpenCV using the Tello camera (does not take off) to draw bounding 
boxes around fiducial markers with coordinates and distance estimation.
'''

from djitellopy import Tello
import cv2
import cv2.aruco as aruco
import numpy as np
import time

# --- Config ---
MARKER_SIZE = 0.105  # 8 cm in meters
ARUCO_DICT = aruco.DICT_4X4_50
TARGET_ID = 0

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

# --- Connect to Tello (make sure you're on its WiFi first) ---
drone = Tello()
drone.connect()
print(f"Battery: {drone.get_battery()}%")

drone.streamon()
time.sleep(2)

# --- Estimated camera calibration ---
# --- Tello camera calibration ---
camera_matrix = np.array([
    [921.170702, 0.000000, 459.904354],
    [0.000000, 919.018377, 351.238301],
    [0.000000, 0.000000, 1.000000]
], dtype=np.float64)
dist_coeffs = np.array([-0.033458, 0.105152, 0.001256, -0.006647, 0.000000], dtype=np.float64)

frame_reader = drone.get_frame_read()

try:
    while True:
        frame = frame_reader.frame
        if frame is None:
            continue

        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)

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

        cv2.imshow("Tello ArUco Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    drone.streamoff()
    cv2.destroyAllWindows()