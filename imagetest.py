import cv2

img = cv2.imread( "2HBS_icn3dloadable (2).png")
height,width,channels = img.shape
print("size: ", width, "x", height)

cv2.imshow("original", img)

cv2.waitKey(0)
cv2.destroyAllWindows()