#!/usr/bin/env python3
from operator import truediv
from pathlib import Path
import rospy
from geometry_msgs.msg import PoseStamped, Twist
import threading

import os

swarm_params = rospy.get_param("/crazyflies")
num_of_drones = len(swarm_params)

cmnd = ""

for i in range(num_of_drones):
    cmnd += "rosrun swarm takeoff {} & ".format(i)

os.system(cmnd)

# try:
#     os.system("bash ~/.ros/start.bash {}".format(num_of_drones))
# except:
#     script = """
#     source devel/setup.bash

#     commandd=""
#     echo "$1"
#     echo "$2"
#     END=$2
#     for((i=$1;i<END;i++))
#     do
#         echo "$i"
#         commandd+="& rosrun swarm takeoff "$1"  "$i" "
#     done
#     echo "$commandd"
#     eval "$commandd"
#     """

#     fle = Path(os.path.expanduser('~/.ros/start.bash'))
#     fle.touch(exist_ok=True)
#     fle.write_text(script)

#     os.system("chmod +x ~/.ros/start.bash")

#     os.system("bash ~/.ros/start.bash {}".format(num_of_drones))
