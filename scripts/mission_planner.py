#!/usr/bin/env python3
import time
import numpy as np
from artificial_potential_field import ArtificialPotentialField
from swarm.srv import FireDataResponse, FireData
import rospy
import os, sys
from utils import array_to_real_positions

field = [[1,1,1,1,1,1,1,1,1,1],
        [1,1,1,1,1,1,1,1,1,1],
        [1,1,1,1,1,1,1,1,1,1],
        [1,1,1,1,1,1,1,1,1,1],
        [1,1,1,1,0,0,0,1,1,1],
        [1,1,1,1,0,0,1,1,1,1],
        [1,1,1,1,1,1,1,1,1,1],
        [1,1,1,1,1,1,1,1,1,1],
        [1,1,1,1,1,1,1,1,1,1],
        [1,1,1,1,1,1,1,1,1,1]]

fielddene = [[1,1,1,1,1],
             [1,0,0,1,1],
             [1,0,0,1,1],
             [1,1,1,1,1]]

sx = 1
sy = 1

width = 1.0
height = 0.8

def firedata_callback(req):
    onedarr = req.grid
    sizex = req.divisionx
    sizey = req.divisiony

    grid = []
    print("Deneme")
    for i in range(sizey):
        arr = []
        for j in range(sizex):
            index = i*sizex+j
            arr.append(onedarr[index])
        
        grid.append(arr)

    print(f"Start x {req.startx}, starty {req.starty}\n\n")
    apf.surround_fire(grid, req.startx/100, req.starty/100, req.width/100, req.height/100, 3)
    return FireDataResponse(True)

srv = rospy.Service("firedata", FireData, firedata_callback)

apf = ArtificialPotentialField()
print("Takeoff")

#[[0, 4, 0], [2.0, 5.0, 0], [4.0, 2.0, 0], [6.0, 4.0, 0], [7.0, 5.0, 0]]
