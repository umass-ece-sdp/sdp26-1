import cv2

def generate_fiducial():
    # Generate ArUco dictionary and create a marker
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

    # Generate marker images (marker ID 0, size 200x200 pixels)
    chest_marker = cv2.aruco.generateImageMarker(aruco_dict, id=0, sidePixels=200)
    left_shoulder_marker = cv2.aruco.generateImageMarker(aruco_dict, id=1, sidePixels=200)
    right_shoulder_marker = cv2.aruco.generateImageMarker(aruco_dict, id=2, sidePixels=200)
    back_marker = cv2.aruco.generateImageMarker(aruco_dict, id=3, sidePixels=200)


    # Save the markers
    cv2.imwrite('software\\lib\\fiducials\\chest_marker.png', chest_marker)
    cv2.imwrite('software\\lib\\fiducials\\left_shoulder_marker.png', left_shoulder_marker)
    cv2.imwrite('software\\lib\\fiducials\\right_shoulder_marker.png', right_shoulder_marker)
    cv2.imwrite('software\\lib\\fiducials\\back_marker.png', back_marker)

if __name__ == '__main__':
    generate_fiducial()