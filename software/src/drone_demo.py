from djitellopy import Tello
import cv2
import cv2.aruco as aruco
import numpy as np
import time

# --- Configuration ---
# Map fiducial IDs to body parts (from fiducials.py)
BODY_PARTS = {
    0: "Chest",
    1: "Left Shoulder",
    2: "Right Shoulder",
    3: "Back"
}

MARKER_SIZE_DEFAULT = 0.2032  # Size in meters for ID 0, 3
MARKER_SIZE_SHOULDER = 0.127 # Size in meters for ID 1, 2
ARUCO_DICT = aruco.DICT_4X4_50

# --- ArUco Setup ---
aruco_dict = aruco.getPredefinedDictionary(ARUCO_DICT)
detector_params = aruco.DetectorParameters()
detector_params.cornerRefinementMethod = aruco.CORNER_REFINE_NONE
detector = aruco.ArucoDetector(aruco_dict, detector_params)

half_default = MARKER_SIZE_DEFAULT / 2.0
OBJ_POINTS_DEFAULT = np.array([
    [-half_default,  half_default, 0],
    [ half_default,  half_default, 0],
    [ half_default, -half_default, 0],
    [-half_default, -half_default, 0],
], dtype=np.float32)

half_shoulder = MARKER_SIZE_SHOULDER / 2.0
OBJ_POINTS_SHOULDER = np.array([
    [-half_shoulder,  half_shoulder, 0],
    [ half_shoulder,  half_shoulder, 0],
    [ half_shoulder, -half_shoulder, 0],
    [-half_shoulder, -half_shoulder, 0],
], dtype=np.float32)

# --- Camera calibration (from better_arcing.py) ---
camera_matrix = np.array([
    [921.170702, 0.000000, 459.904354],
    [0.000000, 919.018377, 351.238301],
    [0.000000, 0.000000, 1.000000]
], dtype=np.float64)
dist_coeffs = np.array([-0.033458, 0.105152, 0.001256, -0.006647, 0.000000], dtype=np.float64)

def main():
    print("Connecting to Tello...")
    drone = Tello()
    drone.connect()
    print(f"Battery: {drone.get_battery()}%")

    print("Starting video stream. Drone will NOT take off.")
    
    # Sometimes the drone thinks it's already streaming. Toggle it off then on.
    drone.streamoff()
    time.sleep(1)
    drone.streamon()
    time.sleep(2)
    
    frame_reader = None
    cap = None

    try:
        print("Attempting to use djitellopy's get_frame_read...")
        frame_reader = drone.get_frame_read()
    except Exception as e:
        print(f"djitellopy get_frame_read failed ({e}). Falling back to native OpenCV VideoCapture workaround...")
        # We use cv2.VideoCapture bypassing av module entirely on problematic machines
        cap = cv2.VideoCapture('udp://@0.0.0.0:11111')
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Minimize frame buffering to reduce lag
        if not cap.isOpened():
            cap = cv2.VideoCapture('udp://0.0.0.0:11111') # try without multicast
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not cap.isOpened():
            print("Failed to open the video stream.")
            return

    print("Press 'q' to quit.")
    
    while True:
        if frame_reader is not None:
            # Using djitellopy frame reader
            frame = frame_reader.frame
            if frame is None:
                continue
            # Convert the native RGB PyAV frame to BGR for OpenCV
            display_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        else:
            # Using fallback cv2 VideoCapture
            ret, display_bgr = cap.read()
            if not ret or display_bgr is None:
                continue
        
        # We just make gray directly  
        gray = cv2.cvtColor(display_bgr, cv2.COLOR_BGR2GRAY)
        
        # Detect markers
        corners, ids, _ = detector.detectMarkers(gray)

        if ids is not None:
            # Draw boundary boxes around the fiducials
            aruco.drawDetectedMarkers(display_bgr, corners, ids)
            
            for corner, marker_id in zip(corners, ids.flatten()):
                # Calculate pose for distance (z) and position (x,y) based on specific marker size
                current_obj_points = OBJ_POINTS_SHOULDER if marker_id in (1, 2) else OBJ_POINTS_DEFAULT
                success, rvec, tvec = cv2.solvePnP(
                    current_obj_points, corner.reshape(4, 2),
                    camera_matrix, dist_coeffs
                )
                
                if success:
                    x, y, z = tvec[0, 0], tvec[1, 0], tvec[2, 0]
                    y = -y # Invert Y axis
                    body_part = BODY_PARTS.get(marker_id, "Unknown")
                    
                    # Pixel coordinates for putting text
                    marker_cx = int(corner[0, :, 0].mean())
                    marker_cy = int(corner[0, :, 1].mean())

                    # Calculate offset from center of frame (where 0,0 is center)
                    frame_h, frame_w = display_bgr.shape[:2]
                    center_offset_x = marker_cx - (frame_w // 2)
                    center_offset_y = (frame_h // 2) - marker_cy # Invert so positive is up

                    # Calculate z in imperial units (feet)
                    z_ft = z * 3.28084

                    # Draw metrics text near the boundary box
                    label = f"ID {marker_id}: {body_part}"
                    metrics_1 = f"Frame offset (px): X: {center_offset_x}, Y: {center_offset_y}"
                    metrics_2 = f"Dist Z: {z:.2f}m ({z_ft:.2f}ft)"
                    
                    # Positioning text slightly above and below the marker center
                    cv2.putText(display_bgr, label, (marker_cx - 60, marker_cy - 40), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    cv2.putText(display_bgr, metrics_1, (marker_cx - 60, marker_cy - 20), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                    cv2.putText(display_bgr, metrics_2, (marker_cx - 60, marker_cy), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

        cv2.imshow("Drone Demo", display_bgr)

        # Quit when 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    print("Cleaning up...")
    drone.streamoff()
    cv2.destroyAllWindows()
    print("Done.")

if __name__ == '__main__':
    main()
