import cv2
import cv2.aruco as aruco
from pathlib import Path
import numpy as np
from typing import Literal

fp = Path(__file__)
dir = fp.parent

class Fiducial:
    MARKER_SIZE = 8.5 * 0.0254   # inches * meters conversion, actual size of printed marker
    ARUCO_DICT = aruco.DICT_4X4_50
    
    def __init__(self, *, recalibrate: bool=False, verbose: bool=False, start_id: int=0, sidePixels: int=400):
        # Define parameters
        self.verbose = verbose
        self.parameters = aruco.DetectorParameters()
        self.aruco_dict = aruco.getPredefinedDictionary(Fiducial.ARUCO_DICT)
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.parameters)

        # Call startup functions
        self.generate_marker(start_id, sidePixels)
        cal_matrix = dir.joinpath('camera_matrix.npy')
        cal_dist = dir.joinpath('dist_coeffs.npy')
        if recalibrate or not cal_matrix.exists() or not cal_dist.exists():
            self.calibrate_camera()
        else:
            self.camera_matrix = np.load(cal_matrix)
            self.dist_coeffs = np.load(cal_dist)

    def generate_marker(self, start_id: int=0, sidePixels: int=400) -> None:
        """
        Generates the fiducial marker needed for tracking the user
        through the drones camera. Generates 4 markers - one per side
        of the target's body. Only needs to be called once (unless 
        another marker set is needed).

        Parameters:
            start_id (int, default=0): The first id used to generate 
                fiducial markers. Iterates through the next 3 id's.
            sidePixels (int, default=400): Number of pixels surrounding
                the fiducial marker in the saved image.
        """

        # Check to see if the markers have already been generated
        ids = range(start_id, start_id + 4)
        existing_stems = {p.stem for p in dir.iterdir()}
        markers_exist = all(f'marker_{i}' in existing_stems for i in ids)
        if markers_exist:
            return

        for id in ids:
            marker = aruco.generateImageMarker(self.aruco_dict, id=id, sidePixels=sidePixels)
            cv2.imwrite(dir.joinpath(f'marker_{id}.png'), marker)

    # TODO: Learn how to calibrate camera
    # TODO: Calibrate for each fiducial, make it smart to know when it hasn't seen one
    def calibrate_camera(self):
        pass

    # TODO: Check frame coloring when being passed in to function, may not be BGR
    def generate_z_cam(self, frame, target_id: int) -> np.ndarray | None:
        """
        Detect all visible ArUco markers and return the [x, y, z] position
        of the marker matching target_id, or None if it is not in frame.

        Parameters:
            frame: BGR image from the drone camera.
            target_id (int): The ArUco marker ID to return a pose for.

        Returns:
            np.ndarray of shape (3,) — [px, py, pz] relative to the camera,
            or None if target_id is not detected in this frame.
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self.detector.detectMarkers(gray)

        if ids is not None:
            half = Fiducial.MARKER_SIZE / 2
            obj_points = np.array([
                [-half,  half, 0],
                [ half,  half, 0],
                [ half, -half, 0],
                [-half, -half, 0],
            ], dtype=np.float32)

            for corner, marker_id in zip(corners, ids.flatten()):
                if marker_id != target_id:
                    continue

                _, rvec, tvec = cv2.solvePnP(
                    obj_points, corner.reshape(4, 2),
                    self.camera_matrix, self.dist_coeffs
                )

                # solvePnP returns tvec as (3, 1); index accordingly
                x, y, z = float(tvec[0, 0]), float(tvec[1, 0]), float(tvec[2, 0])

                if self.verbose: # Print only when desired
                    print(
                        '[FIDUCIAL] Position relative to camera:\n',
                        'Left/Right (m):' + f'{x:>8,.2}\n',
                        'Up/Down (m):' + f'{y:>8,.2}\n',
                        'Forward (m):' + f'{z:>8,.2}\n',
                    )

                # aruco.drawDetectedMarkers(frame, corners)
                # cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)

                return np.array([x, y, z])

        return None




if __name__=='__main__':
    fiducial = Fiducial()