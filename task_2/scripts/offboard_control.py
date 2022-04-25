#!/usr/bin/env python3


'''
This is a boiler plate script that contains hint about different services that are to be used
to complete the task.
Use this code snippet in your code or you can also continue adding your code in the same file


This python file runs a ROS-node of name offboard_control which controls the drone in offboard mode. 
See the documentation for offboard mode in px4 here() to understand more about offboard mode 
This node publishes and subsribes the following topics:

	 Services to be called                   Publications                                          Subscriptions				
	/mavros/cmd/arming                       /mavros/setpoint_position/local                       /mavros/state
	/mavros/set_mode                         /mavros/setpoint_velocity/cmd_vel                     /mavros/local_position/pose   
		 
	
'''

import rospy
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from math import *

class offboard_control:


	def __init__(self):
		#DATA
		self.curr_x = 0
		self.curr_y = 0
		self.curr_z = 0
		self.delta = 0.2

		#NODE
		rospy.init_node('iris_drone', anonymous = True)

		#SUBSCRIBERS
		self.get_pose_subscriber = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.get_pose)
		#PUBLISHERS
		self.publish_pose = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size = 10)
		#SERVICES
		self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
		self.takeoff_service = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
		self.land_service = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
		self.flight_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
		rospy.loginfo('INIT')
	# def __init__(self):
	#     # Initialise rosnode
	#     rospy.init_node('offboard_control', anonymous=True)


	# def takeoff(self, t_alt):
	# 	self.gps_subscriber
	# 	t_lat = self.gps_lat
	# 	t_long = self.gps_long
	# 	rospy.wait_for_service('/mavros/cmd/takeoff')
	# 	try:
	# 		self.takeoff_service(0.0, 0.0, t_lat, t_long, t_alt)
	# 		rospy.loginfo('TAKEOFF')
	# 	except rospy.ServiceException as e:
	# 		rospy.loginfo("Service call failed: " %e)

	def setArm(self):
		# Calling to /mavros/cmd/arming to arm the drone and print fail message on failure
		rospy.wait_for_service('mavros/cmd/arming')  # Waiting untill the service starts 
		try:
			armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool) # Creating a proxy service for the rosservice named /mavros/cmd/arming for arming the drone 
			armService(True)
		except rospy.ServiceException as e:
			print ("Service arming call failed: %s"%e)

		# Similarly delacre other service proxies 

   
	def offboard_set_mode(self):

		rate=rospy.Rate(20)
		#print('OFF')

		rospy.wait_for_service('/mavros/set_mode')
		PS = PoseStamped()

		PS.pose.position.x = 0
		PS.pose.position.y = 0
		PS.pose.position.z = 5



		for i in range(50):
			self.publish_pose.publish(PS)
			rate.sleep()
		try:
			self.flight_mode_service(0, 'OFFBOARD')
			rospy.loginfo('OFFBOARD')


		except rospy.ServiceException as e:
			rospy.loginfo("OFFBOARD Mode could not be set: " %e)

		# Call /mavros/set_mode to set the mode the drone to OFFBOARD
		# and print fail message on failure

	def get_pose(self, location_data):
		
		self.curr_x = location_data.pose.position.x
		self.curr_y = location_data.pose.position.y
		self.curr_z = location_data.pose.position.z

	def set_pose(self, set_x, set_y, set_z):
		update_rate = rospy.Rate(20)
		PS = PoseStamped()
		
		PS.pose.position.x = set_x
		PS.pose.position.y = set_y
		PS.pose.position.z = set_z
		
		distance = sqrt((set_x - self.curr_x)**2 + (set_y - self.curr_y)**2 + (set_y - self.curr_z)**2)
		
		while distance > self.delta:
			
			self.publish_pose.publish(PS)
			self.get_pose_subscriber
			distance = sqrt((set_x - self.curr_x)**2 + (set_y - self.curr_y)**2 + (set_z - self.curr_z)**2)
			update_rate.sleep()
		
		# rospy.loginfo('WAYPOINT REACHED: ' + str(self.waypoint_number))

	def mission_control(self):
		self.setArm()
		self.offboard_set_mode()
		# self.takeoff(10.0)
		#self.set_offboard_mode()
		self.set_pose(0.0, 0.0, 10.0)
		self.set_pose(10, 0.0, 10)
		self.set_pose(10, 10, 10)
		self.set_pose(0, 10, 10)
		self.set_pose(0.0, 0.0, 10.0)
		self.set_pose(0.0, 0.0, 0.0)
	

if __name__ == '__main__':
	try:
		control = offboard_control()
		control.mission_control()
		rospy.spin()
			
	except rospy.ROSInterruptException:
		pass