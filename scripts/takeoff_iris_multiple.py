#!/usr/bin/env python3
from operator import truediv
from pathlib import Path
import rospy
from geometry_msgs.msg import PoseStamped, Twist
import threading

import os

script = """
source devel/setup.bash

commandd="rosrun swarm takeoff 0 "
echo "$1"
END=$1
for((i=1;i<END;i++))
do
    echo "$i"
    commandd+="& rosrun swarm takeoff "$i" "
done
echo "$commandd"
eval "$commandd"
"""

fle = Path(os.path.expanduser('~/.ros/start.bash'))
fle.touch(exist_ok=True)
fle.write_text(script)

rospy.init_node("iris_controller")

swarm_params = rospy.get_param("/crazyflies")

num_of_drones = len(swarm_params)

print("Number of drones: {}".format(num_of_drones))

os.system("chmod +x ~/.ros/start.bash")
os.system("cat ~/.ros/start.bash")

os.system("bash ~/.ros/start.bash {}".format(num_of_drones))
