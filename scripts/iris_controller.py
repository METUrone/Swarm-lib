#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from threading import Thread
import numpy as np

rospy.init_node("iris_controller")

swarm_params = rospy.get_param("/crazyflies")
print(swarm_params)

num_of_drones = len(swarm_params)
sleep_duration = 0.01

