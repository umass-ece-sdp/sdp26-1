"""
Refactor summary:
- Centralized ArUco and camera calibration constants in this module so they are
  the single source of truth shared by both Fiducial and FALCON.
- Updated marker size to 0.213 m and derived object corner points from it.
"""

import cv2
import numpy as np
from pathlib import Path

fp = Path(__file__)
fiducial_dir = fp.parent.joinpath('fiducials')
if not fiducial_dir.exists():
    fiducial_dir.mkdir()

LARGE_MARKER_SIZE_METERS = 0.213
SMALL_MARKER_SIZE_METERS = 0.17
ARUCO_DICT_ID = cv2.aruco.DICT_4X4_50
CAMERA_MATRIX = np.array([
    [921.170702, 0.000000, 459.904354],
    [0.000000, 919.018377, 351.238301],
    [0.000000, 0.000000, 1.000000]
], dtype=np.float64)
DIST_COEFFS = np.array([-0.033458, 0.105152, 0.001256, -0.006647, 0.000000], dtype=np.float64)


class Fiducial:
    LARGE_MARKER_SIZE = LARGE_MARKER_SIZE_METERS
    SMALL_MARKER_SIZE = SMALL_MARKER_SIZE_METERS
    ARUCO_DICT = ARUCO_DICT_ID

    def __init__(self):
        # Initialize marker dictionary and detector
        self.parameters = cv2.aruco.DetectorParameters()
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(Fiducial.ARUCO_DICT)
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)

        # Initialize matrices and vectors
        large_half = Fiducial.LARGE_MARKER_SIZE / 2
        small_half = Fiducial.SMALL_MARKER_SIZE / 2
        self.large_obj_points = np.array([
            [-large_half,  large_half, 0],
            [large_half,  large_half, 0],
            [large_half, -large_half, 0],
            [-large_half, -large_half, 0],
        ], dtype=np.float32)
        self.small_obj_points = np.array([
            [-small_half,  small_half, 0],
            [small_half,  small_half, 0],
            [small_half, -small_half, 0],
            [-small_half, -small_half, 0],
        ], dtype=np.float32)

        # DJI Tello camera calibration for distance (z) measurement
        self.camera_matrix = CAMERA_MATRIX.copy()
        self.dist_coeffs = DIST_COEFFS.copy()

    def generate_marker(self):
        # Generate marker images
        chest_marker = cv2.aruco.generateImageMarker(self.aruco_dict, id=0, sidePixels=400)
        left_shoulder_marker = cv2.aruco.generateImageMarker(self.aruco_dict, id=1, sidePixels=400)
        right_shoulder_marker = cv2.aruco.generateImageMarker(self.aruco_dict, id=2, sidePixels=400)
        back_marker = cv2.aruco.generateImageMarker(self.aruco_dict, id=3, sidePixels=400)

        # Save the markers
        cv2.imwrite(str(fiducial_dir.joinpath('chest_marker.png')), chest_marker)
        cv2.imwrite(str(fiducial_dir.joinpath('left_shoulder_marker.png')), left_shoulder_marker)
        cv2.imwrite(str(fiducial_dir.joinpath('right_shoulder_marker.png')), right_shoulder_marker)
        cv2.imwrite(str(fiducial_dir.joinpath('back_marker.png')), back_marker)

    def get_obj_points_for_target(self, target_id):
        target_id_int = int(target_id)
        if target_id_int in (0, 3):
            return self.large_obj_points
        if target_id_int in (1, 2):
            return self.small_obj_points
        return self.large_obj_points

    def detect_marker(self, frame, target_id):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Convert frame to black and white for better detection
        corners, ids, _ = self.detector.detectMarkers(gray)  # Run detection

        if ids is not None:
            for corner, marker_id in zip(corners, ids.flatten()):
                if marker_id != target_id:
                    continue

                # Calculate x y and z coordinates
                success, rvec, tvec = cv2.solvePnP(
                    self.get_obj_points_for_target(target_id),
                    corner.reshape(4, 2),
                    self.camera_matrix,
                    self.dist_coeffs,
                )
                if not success:
                    continue

                return np.array([tvec[0, 0], tvec[1, 0], tvec[2, 0]])

        return None


if __name__ == '__main__':
    fiducial = Fiducial()
