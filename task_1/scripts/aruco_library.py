#!/usr/bin/env python3
############## Task1.1 - ArUco Detection ##############

import numpy as np
import cv2
from cv2 import aruco
import sys
import math
import time

def detect_ArUco(img):
	## function to detect ArUco markers in the image using ArUco library
	## argument: img is the test image
	## return: dictionary named Detected_ArUco_markers of the format {ArUco_id_no : corners}, where ArUco_id_no indicates ArUco id and corners indicates the four corner position of the aruco(numpy array)
	## 		   for instance, if there is an ArUco(0) in some orientation then, ArUco_list can be like
	## 				{0: array([[315, 163],
	#							[319, 263],
	#							[219, 267],
	#							[215, 167]], dtype=float32)}

	Detected_ArUco_markers = {}

	## enter your code here ##


	# Creates aruco dictionary and identifies marker coordinates for each ArUco tag
	aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
	parameters = aruco.DetectorParameters_create()
	corners, ids, _ = aruco.detectMarkers(img, aruco_dict, parameters = parameters)

	# Stores the markers in the Detected_ArUco_markers dictionary in specified format
	if len(corners)==0:
		return
	for i in range(len(ids)):
		Detected_ArUco_markers[ids[i][0]] = corners[i][0]
	
	return Detected_ArUco_markers


def Calculate_orientation_in_degree(Detected_ArUco_markers):
	## function to calculate orientation of ArUco with respective to the scale mentioned in problem statement
	## argument: Detected_ArUco_markers  is the dictionary returned by the function detect_ArUco(img)
	## return : Dictionary named ArUco_marker_angles in which keys are ArUco ids and the values are angles (angles have to be calculated as mentioned in the problem statement)
	##			for instance, if there are two ArUco markers with id 1 and 2 with angles 120 and 164 respectively, the 
	##			function should return: {1: 120 , 2: 164}

	ArUco_marker_angles = {}

	## enter your code here ##

	for i in Detected_ArUco_markers:

		sumx = 0
		sumy = 0
		for j in range(4):
			sumx += Detected_ArUco_markers[i][j][0]
			sumy += Detected_ArUco_markers[i][j][1]

		# center stores coordinates of centroid of the aruco tag
		center = [sumx//4, sumy//4]

		# mid_point stores coordinates of midpoint of top side
		mid_point = [(Detected_ArUco_markers[i][0][0] + Detected_ArUco_markers[i][1][0]) // 2, (Detected_ArUco_markers[i][0][1] + Detected_ArUco_markers[i][1][1]) // 2]

		delta_x = mid_point[0] - center[0]
		delta_y = center[1] - mid_point[1] #defined acc to the coordinate system of the image

		ArUco_marker_angles[i] = round((math.atan(delta_y/delta_x)) * 180 / math.pi)

		# Adjusts the value returned by arctan function with range [-pi/2, pi/2] to required [0,2pi]
		if delta_x < 0: 
			ArUco_marker_angles[i] = 180 + ArUco_marker_angles[i]
		elif delta_y < 0:
			ArUco_marker_angles[i] += 360

	return ArUco_marker_angles	## returning the angles of the ArUco markers in degrees as a dictionary


def mark_ArUco(img,Detected_ArUco_markers,ArUco_marker_angles):
	## function to mark ArUco in the test image as per the instructions given in problem statement
	## arguments: img is the test image 
	##			  Detected_ArUco_markers is the dictionary returned by function detect_ArUco(img)
	##			  ArUco_marker_angles is the return value of Calculate_orientation_in_degree(Detected_ArUco_markers)
	## return: image namely img after marking the aruco as per the instruction given in problem statement

    ## enter your code here ##
	for i in Detected_ArUco_markers:

		# Declares variable for each corner
		top_left = [int(Detected_ArUco_markers[i][0][0]),int(Detected_ArUco_markers[i][0][1])]
		top_right = [int(Detected_ArUco_markers[i][1][0]), int(Detected_ArUco_markers[i][1][1])]
		bottom_right = [int(Detected_ArUco_markers[i][2][0]), int(Detected_ArUco_markers[i][2][1])]
		bottom_left = [int(Detected_ArUco_markers[i][3][0]), int(Detected_ArUco_markers[i][3][1])]

		# Applies the assigned colour to each corner
		cv2.circle(img, tuple(top_left), 5, [125, 125, 125], -1)
		cv2.circle(img, tuple(top_right), 5, [0, 255, 0], -1)
		cv2.circle(img, tuple(bottom_left), 5, [255, 255, 255], -1)
		cv2.circle(img, tuple(bottom_right), 5, [180, 105, 255], -1)

		sumx = 0
		sumy = 0
		for j in range(4):
			sumx += Detected_ArUco_markers[i][j][0]
			sumy += Detected_ArUco_markers[i][j][1]

		center = [int(round(sumx / 4)), int(round(sumy / 4))]
		mid_point = [int(round((Detected_ArUco_markers[i][0][0] + Detected_ArUco_markers[i][1][0]) / 2)) , int(round((Detected_ArUco_markers[i][0][1] + Detected_ArUco_markers[i][1][1]) / 2))]

		#blue line from center to midpoint of top side
		cv2.line(img, tuple(center), tuple(mid_point), [255, 0, 0], 3)

		#text for indicating ArUco id and angle
		img = cv2.putText(img, str(i), (center[0] + 25, center[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2, cv2.LINE_AA)
		img = cv2.putText(img, str(ArUco_marker_angles[i]), (center[0] - 60, center[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (90, 255, 100), 2, cv2.LINE_AA)

		#team id on the top left
		img = cv2.putText(img, '2863', (7,30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

	return img
