import numpy as np
import cv2
from cv2 import aruco
import sys
import math
import time

img = cv2.imread("/home/rishanjoshua/catkin_ws/src/strawberry_stacker/task_1/scripts/test_image3.png")
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
parameters = aruco.DetectorParameters_create()
corners, ids, _ = aruco.detectMarkers(img, aruco_dict, parameters = parameters)
id = ids[0]
sumx = 0
sumy = 0
print(corners[0][0])
print(ids[0][0])

for j in range(4):
	sumx += corners[0][0][j][0]
	sumy += corners[0][0][j][1]

# center stores coordinates of centroid of the aruco tag
center = [sumx//4, sumy//4]
x = center[0]
y = center[1]

mid_point = [(corners[0][0][0][0] + corners[0][0][1][0]) // 2, (corners[0][0][0][1] + corners[0][0][1][1]) // 2]

delta_x = mid_point[0] - center[0]
delta_y = center[1] - mid_point[1] #defined acc to the coordinate system of the image

angle = round((math.atan(delta_y/delta_x)) * 180 / math.pi)

# Adjusts the value returned by arctan function with range [-pi/2, pi/2] to required [0,2pi]
if delta_x < 0: 
	angle = 180 + angle
elif delta_y < 0:
	angle += 360


