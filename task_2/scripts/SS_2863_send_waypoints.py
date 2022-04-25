#!/usr/bin/env python3


'''
This is a boiler plate script that contains hint about different services that are to be used
to complete the task.
Use this code snippet in your code or you can also continue adding your code in the same file


This python file runs a ROS-node of name waypoint_Mission which sends the waypoints to drone
in mission mode.
This node publishes and subsribes the following topics:

	 Services to be called         Subscriptions				
	/mavros/cmd/arming             /mavros/state
    /mavros/set_mode
    /mavros/mission/push
'''

import rospy
from geometry_msgs.msg import Point, PoseStamped
from mavros_msgs.msg import *
from mavros_msgs.srv import *


class Modes:
    def __init__(self):
        pass

# Calling the rosservices

    # Calling to /mavros/cmd/arming to arm the drone and print fail message on failure
    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')  # Waiting untill the service starts 
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool) # Creating a proxy service for the rosservice named /mavros/cmd/arming for arming the drone 
            armService(True)
        except rospy.ServiceException as e:
            print ("Service arming call failed: %s"%e)

   
    def auto_set_mode(self):
        rospy.wait_for_service('mavros/set_mode')

        setModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
        setModeService(custom_mode="AUTO.MISSION")
        # Call /mavros/set_mode to set the mode the drone to AUTO.MISSION
        # and print fail message on failure
    
    def wpPush(self,index,wps):
        rospy.wait_for_service('mavros/mission/push')
        wpPushService = rospy.ServiceProxy('mavros/mission/push', WaypointPush,persistent=True)
        wpPushService(start_index=0,waypoints=wps)#start_index = the index at which we want the mission to start
        print("Waypoint Pushed")
        # Call /mavros/mission/push to push the waypoints
        # and print fail message on failure
   
class stateMoniter:
    def __init__(self):
        self.state = State()
        # Instantiate a setpoints message
        self.sp = PositionTarget()
        self.sp.type_mask = int('010111111000', 2)
        
    def stateCb(self, msg):
        # Callback function for topic /mavros/state
        self.state = msg

class wpMissionCnt:

    def __init__(self):
        self.wp =Waypoint()
        
    def setWaypoints(self,frame,command,is_current,autocontinue,param1,param2,param3,param4,x_lat,y_long,z_alt):
        self.wp.frame =frame 
        self.wp.command = command 
        self.wp.is_current= is_current
        self.wp.autocontinue = autocontinue 
        self.wp.param1=param1
        self.wp.param2=param2
        self.wp.param3=param3
        self.wp.param4=param4
        self.wp.x_lat= x_lat 
        self.wp.y_long=y_long
        self.wp.z_alt= z_alt 

        return self.wp


def main():

    rospy.init_node('waypoint_Mission', anonymous=True) #Initialise rosnode
    rate = rospy.Rate(20.0)

    stateMt = stateMoniter()
    md = Modes()
    
    rospy.Subscriber("/mavros/state",State, stateMt.stateCb)

    wayp0 = wpMissionCnt()
    wayp1 = wpMissionCnt()
    wayp2 = wpMissionCnt()
    wayp3 = wpMissionCnt()
    wayp4 = wpMissionCnt()
    # Add more waypoints here

    
    wps = [] #List to story waypoints
    
    w = wayp0.setWaypoints(3,22,True,True,0.0,0.0,0.0,float('nan'),19.134641,72.911706,10)
    wps.append(w)

    w = wayp1.setWaypoints(3,16,False,True,0.0,0.0,0.0,float('nan'),19.134617,72.911886,10)
    wps.append(w)

    w = wayp2.setWaypoints(3,16,False,True,0.0,0.0,0.0,float('nan'),19.134434,72.911817,10)
    wps.append(w)
    
    w = wayp3.setWaypoints(3,16,False,True,0.0,0.0,0.0,float('nan'),19.134423,72.911763,10)
    wps.append(w)

    w = wayp4.setWaypoints(3,21,False,True,0.0,0.0,0.0,float('nan'),19.134423,72.911763,0)
    wps.append(w)
    


    print (wps)
    md.wpPush(0,wps)


    # Arming the drone
    while not stateMt.state.armed:
        md.setArm()
        rate.sleep()
        print("ARM!!")

    # Switching the state to auto mode
    while not stateMt.state.mode=="AUTO.MISSION":
        md.auto_set_mode()
        rate.sleep()
        print ("AUTO.MISSION")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass