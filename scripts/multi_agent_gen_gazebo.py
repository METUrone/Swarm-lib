#!/usr/bin/env python3
import rospy
import os

rospy.init_node("multi_agent_gen_gazebo")

swarm_params = rospy.get_param("/crazyflies")
print(swarm_params)

num_of_drones = len(swarm_params)
sleep_duration = 0.01

begin = """<?xml version="1.0"?>
<launch>
    <!-- MAVROS posix SITL environment launch script -->
    <!-- launches Gazebo environment and 2x: MAVROS, PX4 SITL, and spawns vehicle -->
    <!-- vehicle model and world -->
    <arg name="est" default="ekf2"/>
    <arg name="vehicle" default="iris"/>
    <arg name="world" default="$(find mavlink_sitl_gazebo)/worlds/mcmillian_airfield.world"/>
    <!-- gazebo configs -->
    <arg name="gui" default="true"/>
    <arg name="debug" default="false"/>
    <arg name="verbose" default="false"/>
    <arg name="paused" default="false"/>
    <!-- Gazebo sim -->
    <include file="$(find gazebo_ros)/launch/empty_world.launch">
        <arg name="gui" value="$(arg gui)"/>
        <arg name="world_name" value="$(arg world)"/>
        <arg name="debug" value="$(arg debug)"/>
        <arg name="verbose" value="$(arg verbose)"/>
        <arg name="paused" value="$(arg paused)"/>
    </include>
"""

end = """
</launch>
"""

temp = """
<!-- UAV{}-->
    <group ns="uav{}">
        <!-- MAVROS and vehicle configs -->
        <arg name="ID" value="{}"/>
        <arg name="fcu_url" default="udp://:{}@localhost:{}"/>
        <!-- PX4 SITL and vehicle spawn -->
        <include file="$(find px4)/launch/single_vehicle_spawn.launch">
            <arg name="x" value="{}"/>
            <arg name="y" value="{}"/>
            <arg name="z" value="{}"/>
            <arg name="R" value="0"/>
            <arg name="P" value="0"/>
            <arg name="Y" value="0"/>
            <arg name="vehicle" value="$(arg vehicle)"/>
            <arg name="mavlink_udp_port" value="{}"/>
            <arg name="mavlink_tcp_port" value="{}"/>
            <arg name="ID" value="$(arg ID)"/>
        </include>
        <!-- MAVROS -->
        <include file="$(find mavros)/launch/px4.launch">
            <arg name="fcu_url" value="$(arg fcu_url)"/>
            <arg name="gcs_url" value=""/>
            <arg name="tgt_system" value="$(eval 1 + arg('ID'))"/>
            <arg name="tgt_component" value="1"/>
        </include>
    </group>

"""

str=""

for i in range(num_of_drones):
    x = swarm_params[i]["initialPosition"][0]
    y = swarm_params[i]["initialPosition"][1]
    z = swarm_params[i]["initialPosition"][2]
    str = str + temp.format(i, i, i, i+14540, i+14580, x, y, z, i+14560, i+4560)

result = begin + str + end

print(result)


f = open("../PX4-Autopilot/launch/multi_uav_mavros_sitl.launch", "w")
f.write(result)
f.close()
print("success")

os.system("source ../PX4-Autopilot/Tools/setup_gazebo.bash ../PX4-Autopilot/ ../PX4-Autopilot/build/px4_sitl_default")
os.system("export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:../PX4-Autopilot:../PX4-Autopilot/Tools/sitl_gazebo && roslaunch px4 multi_uav_mavros_sitl.launch")