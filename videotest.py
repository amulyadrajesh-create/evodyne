import cv2

capture = cv2.VideoCapture(0)

while capture.isOpened():
    ret, img = capture.read()
    if ( not ret ):
        cv2.waitKey(5)
        continue

    cv2.imshow( "Color", img )

    if ( cv2.waitKey(5) == 27 ):
        break

capture.release()
cv2.destroyAllWindows()