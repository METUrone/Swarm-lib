#!/usr/bin/env python3
from pathlib import Path
from statistics import mode
import rospy
import os

rospy.init_node("multi_agent_gen_gazebo")

swarm_params = rospy.get_param("/crazyflies")
print(swarm_params)
obstacle_params = rospy.get_param("/obstacles")
print(obstacle_params)

num_of_drones = len(swarm_params)
num_of_obstacles = len(obstacle_params)
sleep_duration = 0.01

obstacle_radius = rospy.get_param("/radius")
obstacle_height = rospy.get_param("/height")

model_str = """
<robot name="obstacle">
  <link name="my_obstacle">
    <inertial>
      <origin xyz="2 0 0" />
      <mass value="10.0" />
      <inertia  ixx="5.0" ixy="0.0"  ixz="0.0"  iyy="100.0"  iyz="0.0"  izz="1.0" />
    </inertial>
    <visual>
      <origin xyz="0 0 0"/>
      <geometry>
        <cylinder radius="{}" length="{}"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="2 0 1"/>
      <geometry>
        <box size="1 1 2" />
      </geometry>
    </collision>
  </link>
  <gazebo reference="my_obstacle">
    <material>Gazebo/Blue</material>
  </gazebo>
  </robot>""".format(obstacle_radius, obstacle_height)

f = Path("~/Swarm/src/Swarm-lib/launch/obstacle.urdf").expanduser()
f.write_text(model_str)

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
            <arg name="mavlink_udp_port" value="{}@"/>
            <arg name="mavlink_tcp_port" value="{}@"/>
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
    str = str + temp.format(i, i, i, i+14540, i+14580, x, y, z, i+14580, i+4560)

for i in range(num_of_obstacles):
  obstacle = obstacle_params[i]
  str += '<node name="obstacle_spawn{}" pkg="gazebo_ros" type="spawn_model" output="screen" args="-urdf -file {} -model my_obstacle{}  -x {} -y {} -z {}"/> \n'.format(i, f.absolute(), i, obstacle[0], obstacle[1], obstacle[2])

result = begin + str + end

print(result)


f = open("../PX4-Autopilot/launch/multi_uav_mavros_sitl.launch", "w")
f.write(result)
f.close()
print("success")

os.system("source ../PX4-Autopilot/Tools/setup_gazebo.bash ../PX4-Autopilot/ ../PX4-Autopilot/build/px4_sitl_default")
os.system("export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:../PX4-Autopilot:../PX4-Autopilot/Tools/sitl_gazebo && roslaunch px4 multi_uav_mavros_sitl.launch")