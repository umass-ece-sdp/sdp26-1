import cv2
import numpy as np

class Fiducial:
    MARKER_SIZE = 0.08 # Markers are 8cm x 8cm
    ARUCO_DICT = cv2.aruco.DICT_4X4_50 # 4x4 dictionary to have best tracking
    
    def __init__(self):
        # Initialize marker dictionary and detector
        self.parameters = cv2.aruco.DetectorParameters()
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(Fiducial.ARUCO_DICT)
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)

        # Initialize matrices and vectors
        half = Fiducial.MARKER_SIZE / 2
        self.obj_points = np.array([
            [-half,  half, 0],
            [ half,  half, 0],
            [ half, -half, 0],
            [-half, -half, 0],
        ], dtype=np.float32) # Matrix for corner coordinates of fiducial

        # DJI Tello camera calibration for distance (z) measurement
        self.camera_matrix = np.array([
            [921.170702, 0.000000, 459.904354],
            [0.000000, 919.018377, 351.238301],
            [0.000000, 0.000000, 1.000000]
        ], dtype=np.float64) 
        self.dist_coeffs = np.array([-0.033458, 0.105152, 0.001256, -0.006647, 0.000000], dtype=np.float64)

    def generate_marker(self):
        # Generate marker images
        chest_marker = cv2.aruco.generateImageMarker(self.aruco_dict, id=0, sidePixels=400)
        left_shoulder_marker = cv2.aruco.generateImageMarker(self.aruco_dict, id=1, sidePixels=400)
        right_shoulder_marker = cv2.aruco.generateImageMarker(self.aruco_dict, id=2, sidePixels=400)
        back_marker = cv2.aruco.generateImageMarker(self.aruco_dict, id=3, sidePixels=400)

        # Save the markers
        cv2.imwrite('software\\lib\\fiducials\\chest_marker.png', chest_marker)
        cv2.imwrite('software\\lib\\fiducials\\left_shoulder_marker.png', left_shoulder_marker)
        cv2.imwrite('software\\lib\\fiducials\\right_shoulder_marker.png', right_shoulder_marker)
        cv2.imwrite('software\\lib\\fiducials\\back_marker.png', back_marker)

def detect_marker(self, frame, target_id):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # Convert frame to black and white for better detection
    corners, ids, _ = self.detector.detectMarkers(gray) # Run detection

    if ids is not None:
        for corner, marker_id in zip(corners, ids.flatten()):
            if marker_id != target_id:
                continue

            # Calculate x y and z coordinates
            success, rvec, tvec = cv2.solvePnP(
                self.obj_points, corner.reshape(4, 2),
                self.camera_matrix, self.dist_coeffs
            )
            if not success:
                continue

            return np.array([tvec[0, 0], tvec[1, 0], tvec[2, 0]])

    return None

if __name__=='__main__':
    fiducial = Fiducial()