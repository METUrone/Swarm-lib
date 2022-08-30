#!/usr/bin/env python3
import imp
import rospy

from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import * 
from mavros_msgs.srv import * 
from swarm.srv import *

import sys

command_state = "pose"

pose = PoseStamped()
pose.pose.position.x = 0
pose.pose.position.y = 0
pose.pose.position.z = 5

velocity = TwistStamped()

global state 

def position_command(req):
    global command_state, pose
    command_state = "pose"

    pose.pose.position.x = req.x
    pose.pose.position.y = req.y
    pose.pose.position.z = req.z
    return PoseCommandResponse(True)

def velocity_command(req):
    global command_state, velocity
    command_state = "velocity"

    velocity.twist.linear.x = req.linear_x
    velocity.twist.linear.y = req.linear_y
    velocity.twist.linear.z = req.linear_z
    velocity.twist.angular.x = req.angular_x
    velocity.twist.angular.y = req.angular_y
    velocity.twist.angular.z = req.angular_z
    return VelocityCommandResponse(True)
    

def state_callback(data):
    
    global state
    state = data
    return

def state_listener():
    rospy.Subscriber("/uav{}/mavros/state".format(0), State, state_callback)
    rate = rospy.Rate(100)
    rate.sleep()



if __name__ == '__main__':
    
    rospy.init_node("takeoff{}".format(0), anonymous=True)
    rate = rospy.Rate(10)

    rospy.wait_for_service("/uav{}/mavros/cmd/arming".format(0))
    arming_client = rospy.ServiceProxy("/uav{}/mavros/cmd/arming".format(0), CommandBool)

    rospy.wait_for_service("/uav{}/mavros/set_mode".format(0)) 
    set_mode_client = rospy.ServiceProxy("/uav{}/mavros/set_mode".format(0), SetMode)

    command_service = rospy.Service("PoseCommand{}".format(0), PoseCommand, position_command)
    velocity_service = rospy.Service("VelocityCommand{}".format(0), VelocityCommand, velocity_command)

    pose_pub = rospy.Publisher("/uav{}/mavros/setpoint_position/local".format(0), PoseStamped, queue_size=10)
    pose_pub.publish(pose)

    velocity_pub = rospy.Publisher("/uav{}/mavros/setpoint_velocity/cmd_vel".format(0), TwistStamped, queue_size=1000)

    set_mode = SetMode()
    set_mode._response_class.custom_mode = "OFFBOARD"
    set_mode._response_class.base_mode = 0

    state_listener()

    for i in range(100):
        pose_pub.publish(pose)
        state_listener()

    if state.connected:
        print("CONNECTED") 
    else:
        print("CONNECTION FAILED")

    last_request = rospy.Time.now()

    try:

        while not rospy.is_shutdown():
            if state.mode != "OFFBOARD" and (rospy.Time.now() - last_request > rospy.Duration(5.0)):
                if set_mode_client(base_mode=0, custom_mode="OFFBOARD") and state.mode == "OFFBOARD":
                    print("OFFBOARD")
                    last_request = rospy.Time.now()
            else:
                if not state.armed and (rospy.Time.now()-last_request > rospy.Duration(5.0)):
                    arming_client(True)
                    last_request = rospy.Time.now()

            if command_state == "pose":
                pose_pub.publish(pose)
            elif command_state == "velocity":
                velocity_pub.publish(velocity)
            
            rate.sleep()

    except rospy.exceptions.ROSInterruptException:
        print("\nshutdown")