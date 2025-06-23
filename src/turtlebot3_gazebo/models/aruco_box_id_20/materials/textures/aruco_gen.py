import cv2
import cv2.aruco

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
img = cv2.aruco.generateImageMarker(aruco_dict, 20, 200)  # ID=25, size=200x200 px
cv2.imwrite("aruco_20.png", img)
