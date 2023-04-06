#!/usr/bin/env python3
from operator import truediv
from time import sleep
import rospy
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from sensor_msgs.msg import NavSatFix
from threading import Thread
import numpy as np


sim_origin_latitude: 47.3977419
sim_origin_longitude: 8.545594
sim_origin_altitude: 535.2329155639569


rospy.init_node("iris_controller")

swarm_params = rospy.get_param("/crazyflies")

num_of_drones = len(swarm_params)
sleep_duration = 0.01

agent_ids = [param["id"] for param in swarm_params]
pose_publishers = {}
vel_publishers = {}
vel_commands = {}
current_poses = {}

min_agent_id = min(agent_ids)

def pose_callback(data : PoseStamped, agent_id):
    global current_poses
    global min_agent_id
    current_poses[agent_id].pose.position.x = data.pose.position.x + swarm_params[agent_id - min_agent_id]["initialPosition"][0]
    current_poses[agent_id].pose.position.y = data.pose.position.y + swarm_params[agent_id - min_agent_id]["initialPosition"][1]
    current_poses[agent_id].pose.position.z = data.pose.position.z + swarm_params[agent_id - min_agent_id]["initialPosition"][2]

def vel_commander_callback(data : Twist, agent_id):
    global vel_commands
    vel_commands[agent_id] = data


def send_vel_command(agent_id):
    global vel_commands
    vel_command = vel_commands[agent_id]
    vel_publishers[agent_id].publish(vel_command)
    rospy.sleep(sleep_duration)

for agent_id in agent_ids:
    pose_publishers[agent_id] = rospy.Publisher("/{}/position".format(agent_id), PoseStamped, queue_size=100)
    rospy.Subscriber("/uav{}/mavros/local_position/pose".format(agent_id-min_agent_id), PoseStamped, pose_callback, callback_args=agent_id)
    rospy.Subscriber("/{}/vel_commander".format(agent_id), Twist, vel_commander_callback, callback_args=agent_id)
    vel_publishers[agent_id] = rospy.Publisher("/uav{}/mavros/setpoint_velocity/cmd_vel_unstamped".format(agent_id-min_agent_id), Twist, queue_size=100)

    vel_commands[agent_id] = Twist()
    vel_commands[agent_id].linear.x = 0
    vel_commands[agent_id].linear.y = 0
    vel_commands[agent_id].linear.z = 0
    vel_commands[agent_id].angular.x = 0
    vel_commands[agent_id].angular.y = 0
    vel_commands[agent_id].angular.z = 0

    current_poses[agent_id] = PoseStamped()
    current_poses[agent_id].pose.position.x = 0
    current_poses[agent_id].pose.position.y = 0
    current_poses[agent_id].pose.position.z = 0


def publish_positions(agent_ids):
    pose = PoseStamped()

    while not rospy.is_shutdown():
        global sleep_duration
        global sim_origin_latitude
        global sim_origin_longitude
        global sim_origin_altitude 
        for agent_id in agent_ids:
            agent_pose = current_poses[agent_id]
            pose.pose.position.x = agent_pose.pose.position.x
            pose.pose.position.y = agent_pose.pose.position.y
            pose.pose.position.z = agent_pose.pose.position.z

            pose_publishers[agent_id].publish(pose)

        rospy.sleep(sleep_duration)


def send_vel_commands(agent_ids):

    global sleep_duration 

    while not rospy.is_shutdown():
        for agent_id in agent_ids:
            send_vel_command(agent_id)
            rospy.sleep(sleep_duration)

        rospy.sleep(sleep_duration)

Thread(target=rospy.spin).start()
Thread(target=publish_positions, args=(agent_ids,)).start()
Thread(target=send_vel_commands, args=(agent_ids,)).start()

while not rospy.is_shutdown():
    rospy.sleep(sleep_duration)




