#!/usr/bin/env python3


'''
This is a boiler plate script that contains an example on how to subscribe a rostopic containing camera frames 
and store it into an OpenCV image to use it further for image processing tasks.
Use this code snippet in your code or you can also continue adding your code in the same file


This python file runs a ROS-node of name marker_detection which detects a moving ArUco marker.
This node publishes and subsribes the following topics:

	Subsriptions					Publications
	/camera/camera/image_raw			/marker_info
'''
from sensor_msgs import msg
from sensor_msgs.msg import Image
from task_1.msg import Marker
from cv_bridge import CvBridge, CvBridgeError
import cv2
import math
from cv2 import aruco
import numpy as np
import rospy


class image_proc():

	# Initialise everything
	def __init__(self):
		rospy.init_node('marker_detection') #Initialise rosnode 
		self.id = 0
		self.x = 0
		self.y = 0
		self.z = 0
		self.roll = 0
		self.pitch = 0
		self.yaw = 0
		
		# Making a publisher 
		
		self.marker_pub = rospy.Publisher('/marker_info', Marker, queue_size=1)
		
		# ------------------------Add other ROS Publishers here-----------------------------------------------------
	
        	# Subscribing to /camera/camera/image_raw

		self.image_sub = rospy.Subscriber("/camera/camera/image_raw", Image, self.image_callback) #Subscribing to the camera topic
		
	        # -------------------------Add other ROS Subscribers here----------------------------------------------------
        
		self.img = np.empty([]) # This will contain your image frame from camera
		self.bridge = CvBridge()
		
		self.marker_msg=Marker()  # This will contain the message structure of message type task_1/Marker

	def processing(self):
		self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
		self.parameters = aruco.DetectorParameters_create()
		self.corners, self.ids, _ = aruco.detectMarkers(self.img, self.aruco_dict, parameters = self.parameters)
		self.id = self.ids[0][0]
		sumx = 0
		sumy = 0
		for j in range(4):
			sumx += self.corners[0][0][j][0]
			sumy += self.corners[0][0][j][1]

		# center stores coordinates of centroid of the aruco tag
		center = [sumx//4, sumy//4]
		self.x = center[0]
		self.y = center[1]

		mid_point = [(self.corners[0][0][0][0] + self.corners[0][0][1][0]) // 2, (self.corners[0][0][0][1] + self.corners[0][0][1][1]) // 2]

		delta_x = mid_point[0] - center[0]
		delta_y = center[1] - mid_point[1] #defined acc to the coordinate system of the image

		angle = round((math.atan(delta_y/(delta_x+0.01))) * 180 / math.pi)

		# Adjusts the value returned by arctan function with range [-pi/2, pi/2] to required [0,2pi]
		if delta_x < 0: 
			angle = 180 + angle
		elif delta_y < 0:
			angle += 360

		self.yaw = angle

		return

	# Callback function of camera topic
	def image_callback(self, data):
		self.rate = rospy.Rate(10)
		self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
		self.processing()
		self.publish_data()
		self.rate.sleep()

		# Note: Do not make this function lenghty, do all the processing outside this callback function
		try:
			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image
		except CvBridgeError as e:
			print(e)
			return
			
	def publish_data(self):
		self.marker_msg.x= self.x
		self.marker_msg.y= self.y
		self.marker_msg.id= self.id
		self.marker_msg.yaw= self.yaw

		self.marker_pub.publish(self.marker_msg)

if __name__ == '__main__':
    image_proc_obj = image_proc()
    rospy.spin()
