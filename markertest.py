import cv2

dictionary = cv2.aruco.getPredefinedDictionary ( cv2.aruco.DICT_6X6_250 )
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector( dictionary, parameters )

capture = cv2.VideoCapture(0)

while capture.isOpened():
    ret, img = capture.read()
    if ( not ret ):
        cv2.waitKey(5)
        continue

    marker_corners, marker_ids, rejected = detector.detectMarkers( img )
    print( marker_corners )
    print( marker_ids )
    cv2.aruco.drawDetectedMarkers( img, marker_corners )

    cv2.imshow( "Color", img )

    if ( cv2.waitKey(5) == 27 ):
        break

capture.release()
cv2.destroyAllWindows()