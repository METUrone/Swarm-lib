#!/usr/bin/env python3
import time, rospy
import numpy as np
from artificial_potential_field import ArtificialPotentialField
from swarm.srv import FireData, FireDataResponse

apf = ArtificialPotentialField()
apf.form_via_potential_field(2)

def firedata_callback(req):
    onedarr = req.grid 
    size = req.division

    field = []

    for i in range(size):
        arr = []
        for j in range(size):
            index = i*size+j
            arr.append(onedarr[index])
        
        field.append(arr)

    apf.surround_fire(field)
    return FireDataResponse(True)

srv = rospy.Service("firedata", FireData, firedata_callback)
