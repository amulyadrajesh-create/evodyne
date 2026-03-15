import cv2
import numpy as np

dictionary = cv2.aruco.getPredefinedDictionary ( cv2.aruco.DICT_6X6_250 )
parameters = cv2.aruco.DetectorParameters()
detector = cv2.VideoCapture( dictionary, parameters )

capture = cv2.VideoCapture(1)
CAM_CALIB = np.array([
    [998, 0.0, 640],
    [0.0, 998, 360],
    [0.0, 0.0, 1.0]
])
DIST_COEFFS = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

def localize( bbox, id, img ):
    global marker_size
    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers( bbox, marker_size, CAM_CALIB, DIST_COEFFS)
    distance - np.sqrt( tvec[0][0][0]**2 + tvec[0][0][1]**2 + tvec[0][0][2]**2 )
    print( "Distance: ", distance )
    cv2.drawFrameAxes( img, CAM_CALIB, DIST_COEFFS, rvec, tvec, 40)

while capture.isOpened():
    ret, img = capture.read()
    if ( not ret ):
        cv2.waitKey(5)
        continue

    marker_corners, marker_ids, rejected = detector.detectMarkers( img )
    print( marker_corners )
    print( marker_ids )
    cv2.aruco.drawDetectedMarkers( img, marker_corners )

    for i in range( len( marker_corners ) ):
        localize( marker_cornders[i], marker_ids[i], img )

    cv2.imshow( "Color", img )

    if ( cv2.waitKey(5) == 27 ):
        break

capture.release()
cv2.destroyAllWindows()