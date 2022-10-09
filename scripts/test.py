#!/usr/bin/env python3

import rospy
import sys
from geometry_msgs.msg import Twist

def make_circle(linear_vel, angular_vel):
  rospy.init_node("make_circle", anonymous = True)
  publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=100)
  rate = rospy.Rate(10)

  vel = Twist()
  
  vel.linear.x, vel.linear.y, vel.linear.z= linear_vel, 0, 0
  vel.angular.x, vel.angular.y, vel.angular.z = 0, 0, angular_vel


  while True:
    publisher.publish(vel)
    rate.sleep()

make_circle(1, 1)