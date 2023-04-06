#!/usr/bin/env python3

import sys
import os
username = os.getlogin()
sys.path.append(f"/home/{username}/crazyswarm/ros_ws/src/crazyswarm/scripts")

import rospy
from pycrazyswarm import Crazyswarm
from geometry_msgs.msg import PoseStamped, Twist
from threading import Thread
import numpy as np

if sys.argv[1] == "--sim":
    rospy.init_node("crazyswarm_controller")

swarm_params = rospy.get_param("/crazyflies")
print(rospy.get_param("/crazyflies")[0]["id"])

num_of_drones = len(swarm_params)
sleep_duration = 0.01


crazyswarm = Crazyswarm("{}/../config/crazyflies.yaml".format(__file__.strip("crazyswarm_controller.py")))
agent_list = crazyswarm.allcfs.crazyflies
timeHelper = crazyswarm.timeHelper
agents_by_id = crazyswarm.allcfs.crazyfliesById
pose_publishers = {}
vel_commands = {}

for agent in agent_list:
    agent.takeoff(targetHeight=0.8, duration=1)
timeHelper.sleep(4)


def vel_commander_callback(data : Twist, agent_id):
    global vel_commands
    vel_commands[agent_id] = np.array([data.linear.x, data.linear.y, data.linear.z])


for agent_id in agents_by_id.keys():
    pose_publishers[agent_id] = rospy.Publisher("/{}/position".format(agent_id), PoseStamped, queue_size=100)
    rospy.Subscriber("/{}/vel_commander".format(agent_id), Twist, vel_commander_callback, callback_args=agent_id)
    vel_commands[agent_id] = np.array([0.0, 0.0, 0.0])

def publish_positions(agents_by_id):
    pose = PoseStamped()
    global timeHelper

    while not rospy.is_shutdown():
    
        global sleep_duration 

        for agent_id in agents_by_id.keys():
            agent_pose = agents_by_id[agent_id].position()
            pose.pose.position.x = agent_pose[0]
            pose.pose.position.y = agent_pose[1]
            pose.pose.position.z = agent_pose[2]

            pose_publishers[agent_id].publish(pose)
        
        rospy.sleep(sleep_duration)


def send_vel_commands(agents_by_id):

    global sleep_duration 

    while not rospy.is_shutdown():
        for agent_id in agents_by_id.keys():
            agents_by_id[agent_id].cmdVelocityWorld(vel_commands[agent_id], 0.0)

        rospy.sleep(sleep_duration)

Thread(target=rospy.spin).start()
Thread(target=publish_positions, args=(agents_by_id,)).start()
Thread(target=send_vel_commands, args=(agents_by_id,)).start()

while not rospy.is_shutdown():
    timeHelper.sleep(sleep_duration)




