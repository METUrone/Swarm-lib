#!/usr/bin/env python3

# -*- coding: utf-8 -*-

import math
import time
import sys
import os, signal
import rospy
from geometry_msgs.msg import PoseStamped, Twist

import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.log import LogConfig
from threading import Event, Thread


deck_attached_event = Event()


rospy.init_node("crazyswarm_controller")

swarm_params = rospy.get_param("/crazyflies")

uris = []
positions = {}
ids = []
pose_publishers = {}
vel_commands = {}
battery_voltages = {}
battery_level = {}

def vel_callback_handler(id):
    def vel_callback(data, id=id):
        global vel_commands
        print(vel_commands)
        vel_commands[id] = data
        
    return vel_callback

for param in swarm_params:
    pose = PoseStamped()

    uris.append('radio://0/{}/2M/E7E7E7E7{}'.format(param["channel"], param["id"]))
    pose.pose.position.x = param["initialPosition"][0]
    pose.pose.position.y = param["initialPosition"][1]
    pose.pose.position.z = param["initialPosition"][2]

    positions[param["id"]] = pose
    ids.append(param["id"])
    pose_publishers[param["id"]] = rospy.Publisher("/{}/pose".format(param["id"]), PoseStamped, queue_size=1)

    vel_commands[param["id"]] = Twist()
    rospy.Subscriber("/{}/vel_commander".format(param["id"]), Twist, vel_callback_handler(param["id"]))



def pos_callback_handler(id):
    def log_pos_callback(timestamp, data, logconf, id=id):
        global positions, pose_publishers, battery_voltages, battery_level


        positions[id].header.stamp = rospy.Time.now()
        positions[id].pose.position.x = data['stateEstimate.x']
        positions[id].pose.position.y = data['stateEstimate.y']
        positions[id].pose.position.z = data['stateEstimate.z']
        battery_voltages[id] = data['pm.vbat']
        battery_level[id] = data['pm.batteryLevel']




        pose_publishers[id].publish(positions[id])
        rospy.sleep(0.01)


    return log_pos_callback

def cf_loop(uri):
    global vel_commands

    
    cf_id = uri[-2:]

    duration = 2.0
    takeoff_rate = 10.0
    for i in range(int(duration *  takeoff_rate)):
        swarm._cfs[uri].cf.commander.send_hover_setpoint(0, 0, 0, 0.2) # takeoff
        time.sleep(1.0 / takeoff_rate)


    for _ in range(5 * 10):

        swarm._cfs[uri].cf.commander.send_velocity_world_setpoint(vel_commands[cf_id].linear.x, vel_commands[cf_id].linear.y, vel_commands[cf_id].linear.z, 0)
        time.sleep(0.1)




    swarm._cfs[uri].cf.commander.send_hover_setpoint(0, 0, 0, 0.1) # land


def start_logging_for_all():
    for uri in uris:
        cf = swarm._cfs[uri].cf
        logconf = LogConfig(name='Position', period_in_ms=10)
        logconf.add_variable('stateEstimate.x', 'float')
        logconf.add_variable('stateEstimate.y', 'float')
        logconf.add_variable('stateEstimate.z', 'float')
        logconf.add_variable('pm.vbat', 'FP16')
        logconf.add_variable('pm.batteryLevel', 'float')


        cf.log.add_config(logconf)
        cf_id = uri[-2:]

        logconf.data_received_cb.add_callback(pos_callback_handler(cf_id))

        logconf.start()

def check_battery_status():
    rospy.loginfo("Battery Status")
    global battery_voltages, battery_level

    for _ in range(10):
            if battery_voltages == {}:
                rospy.sleep(0.2)

    for id in ids:
        rospy.loginfo("ID: {}  Voltage: {} Level: {}/100".format(id, battery_voltages[id], battery_level[id]))

    start_mission = input("Start mission? (y/n): ")

    if start_mission == "n":
        try:
            sys.exit(0)
        except:
            os._exit(0)

def start_missions():
    threads = []
    for uri in uris:
        t = Thread(target=cf_loop, args=(uri,))
        t.start()
        threads.append(t)

    for t in threads:
        t.join()


if __name__ == '__main__':

    cflib.crtp.init_drivers()


    factory = CachedCfFactory(rw_cache='./cache')
    

    with Swarm(uris, factory=factory) as swarm:


        start_logging_for_all()
        check_battery_status()


        # start_missions()


        sys.exit(-1)