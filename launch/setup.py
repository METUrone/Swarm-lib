#!/usr/bin/env python3

import rospy
import yaml
import os

params=rospy.get_param("crazyflies") # Gets params from crazyflies parameters
params_dict = {"crazyflies": params}

#updates crazyflies yaml file with new parameters
with open('/home/{}/crazyswarm/ros_ws/src/crazyswarm/launch/crazyflies.yaml'.format(os.getlogin()), 'w') as outfile:
    yaml.dump(params_dict, outfile, default_flow_style=False)