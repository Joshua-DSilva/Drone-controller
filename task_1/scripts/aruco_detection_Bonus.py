#!/usr/bin/env python3

############## Task1.1 - ArUco Detection ##############
### YOU CAN EDIT THIS FILE FOR DEBUGGING PURPOSEs, SO THAT YOU CAN TEST YOUR ArUco_library.py AGAINST THE VIDEO Undetected ArUco markers.avi###
### BUT MAKE SURE THAT YOU UNDO ALL THE CHANGES YOU HAVE MADE FOR DEBUGGING PURPOSES BEFORE TESTING AGAINST THE TEST IMAGES ###

import numpy as np
import cv2
# import cv2.aruco as ArUco
import time
from aruco_library import *


cap = cv2.VideoCapture('/home/rishanjoshua/catkin_ws/src/strawberry_stacker/task_1/scripts/video2.mp4')
if (cap.isOpened()== False): 
  print("Error opening video  file")

frame_width = int(cap.get(3))
frame_height = int(cap.get(4))
   
size = (frame_width, frame_height)
   
# Below VideoWriter object will create
# a frame of above defined The output 
# is stored in 'filename.avi' file.
result = cv2.VideoWriter('/home/rishanjoshua/catkin_ws/src/strawberry_stacker/task_1/scripts/2863_Task1_bonus.mp4', 
                         cv2.VideoWriter_fourcc(*'MJPG'),
                         25, size)

while True:
	# Capture frame-by-frame
	ret, frame = cap.read()

	# print(len(frame)) 
	# if frame is read correctly ret is True
	if not ret:
		print("Can't receive frame (stream end?). Exiting ...")
		break
	# Our operations on the frame come here
	frame1 = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	# th, frame1 = cv2.threshold(frame1, 120, 255, cv2.THRESH_BINARY)


	if detect_ArUco(frame1)!=None:
		Detected_ArUco_markers = detect_ArUco(frame1)	
		angle = Calculate_orientation_in_degree(Detected_ArUco_markers)	
		frame = mark_ArUco(frame,Detected_ArUco_markers,angle)			
					
	# Display the resulting frame
	result.write(frame)
	cv2.imshow('frame', frame)
	cv2.imshow('thresh', frame1)
	if cv2.waitKey(1) == ord('q'):
		break
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
