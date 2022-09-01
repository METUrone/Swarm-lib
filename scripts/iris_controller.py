#!/usr/bin/env python3
from time import sleep
import rospy
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from sensor_msgs.msg import NavSatFix
from threading import Thread
import numpy as np

latitude: 47.3977419
longitude: 8.5455938
altitude: 535.2649158429224


rospy.init_node("iris_controller")

swarm_params = rospy.get_param("/crazyflies")
print(rospy.get_param("/crazyflies")[0]["id"])

num_of_drones = len(swarm_params)
sleep_duration = 0.01

agent_ids = [param["id"] for param in swarm_params]
pose_publishers = {}
vel_publishers = {}
vel_commands = {}
current_poses = {}

def pose_callback(data : PoseStamped, agent_id):
    global current_poses
    current_poses[agent_id] = data

def vel_commander_callback(data : Twist, agent_id):
    global vel_commands
    vel_commands[agent_id] = data


def send_vel_command(agent_id):
    global vel_commands
    vel_command = vel_commands[agent_id]
    vel_publishers[agent_id].publish(vel_command)
    rospy.sleep(sleep_duration)

min_agent_id = min(agent_ids)

for agent_id in agent_ids:
    pose_publishers[agent_id] = rospy.Publisher("/{}/position".format(agent_id), PoseStamped, queue_size=100)
    rospy.Subscriber("/uav{}/mavros/global_position/global".format(agent_id-min_agent_id), NavSatFix, pose_callback, callback_args=agent_id)
    rospy.Subscriber("/{}/vel_commander".format(agent_id), Twist, vel_commander_callback, callback_args=agent_id)
    vel_publishers[agent_id] = rospy.Publisher("/uav{}/mavros/setpoint_velocity/cmd_vel_unstamped".format(agent_id-min_agent_id), Twist, queue_size=100)

    vel_commands[agent_id] = Twist()
    vel_commands[agent_id].linear.x = 0
    vel_commands[agent_id].linear.y = 0
    vel_commands[agent_id].linear.z = 0
    vel_commands[agent_id].angular.x = 0
    vel_commands[agent_id].angular.y = 0
    vel_commands[agent_id].angular.z = 0

    current_poses[agent_id] = NavSatFix()
    current_poses[agent_id].longitude = 0
    current_poses[agent_id].latitude = 0
    current_poses[agent_id].altitude = 0

def publish_positions(agent_ids):
    pose = PoseStamped()
    global timeHelper

    while not rospy.is_shutdown():
    
        global sleep_duration 

        for agent_id in agent_ids:
            agent_pose = current_poses[agent_id]
            pose.pose.position.x = agent_pose.longitude - sim_origin_longitude
            pose.pose.position.y = agent_pose.latitude - sim_origin_latitude
            pose.pose.position.z = agent_pose.altitude - sim_origin_altitude

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




