import math
import csv
import matplotlib.pyplot as plt
import numpy as np
from math import sin, cos

from mpl_toolkits.mplot3d import Axes3D

def angle_of_vector(x, y):
    angle = math.atan(abs(y)/abs(x))

    if x >= 0 and y >= 0:
        return angle
    elif x < 0 and y >= 0:
        return math.pi - angle
    elif x < 0 and y < 0:
        return math.pi + angle
    else:
        return 2*math.pi - angle

# Input is distance between agents and output is the distance to the center of the formation. It uses cos theorem.
def distance_to_radius(distance, number_of_edge):
    try:
        d = abs(distance**2/(2*(1 - math.cos(2*math.pi/number_of_edge))))**(0.5)
    except:
        d = 0
    return d 

# Converst degree to radian
def degree_to_radian(angle):
    return angle*0.0174532925 #radians 

# Plots coordinates using matplotlib
# Dimension parameter determines wether the plot is going to be 2 dimensional or 3
def show_coordinates(coordinates, dimension=3):

    X = []
    Y = []
    Z = []

    for i in coordinates:
        X.append(i[0])
        Y.append(i[1])
        Z.append(i[2])

    if dimension == 3:

        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        ax.scatter(X,Y, Z)

    elif dimension == 2:

        plt.scatter(X,Y)
        
    plt.show()

def formation_coordinates(distance_between, num_of_edges, height = 1, displacement=np.array([0,0,0]) ,rotation_angle=0):
    vectors = np.zeros(shape=(num_of_edges,3)) # [id][0: for x, 1: for y, 2: for z]
    num_of_edges = num_of_edges
    radius = distance_to_radius(distance_between, num_of_edges)
    vectors[0][0]=radius
    vectors[0][1]=0
    vectors[0][2]=height

    angle = degree_to_radian(360/num_of_edges)
    agent_angle = 0

    for i in range(num_of_edges):
        agent_angle = i*angle + degree_to_radian(rotation_angle)
        vectors[i][0] = math.floor((radius*math.cos(agent_angle) + displacement[0]) * 1000)/ 1000
        vectors[i][1] = math.floor((radius*math.sin(agent_angle) + displacement[1]) * 1000)/ 1000
        vectors[i][2] = math.floor((height + displacement[2]) * 1000)/ 1000

    return vectors

def rotate_coordinates(coordinates, angle):
    angle = degree_to_radian(angle)
    center = [0,0,0]
    result = []

    rot = np.array([[cos(angle), -sin(angle)], [sin(angle), cos(angle)]])

    for coordinate in coordinates:
        center[0] = center[0] + coordinate[0]
        center[1] = center[1] + coordinate[1]
    
    center = [center[0]/len(coordinates), center[1]/len(coordinates)]
    
    coordinates_respect_to_center = []

    for coordinate in coordinates:
        coordinates_respect_to_center.append([coordinate[0]- center[0], coordinate[1]- center[1]])

    for i in range(len(coordinates_respect_to_center)):
        result.append(  [np.dot(rot, coordinates_respect_to_center[i])[0] + center[0], np.dot(rot, coordinates_respect_to_center[i])[1] + center[1], coordinates[i][2]])    

    return result


def rotate_coordinates_wrt_to(coordinates,angle,point=np.zeros(2)):
    angle = degree_to_radian(angle)
    coordinates_respect_to_point=[]
    rot=np.array([[cos(angle), -sin(angle)], [sin(angle), cos(angle)]])
    result=[]
    for coordinate in coordinates:
        coordinates_respect_to_point.append([coordinate[0]- point[0], coordinate[1]- point[1]])
    for i in range(len(coordinates_respect_to_point)):
        tmp_point=(np.dot(rot, coordinates_respect_to_point[i])+point)
        result.append([tmp_point[0],tmp_point[1],coordinates[i][2]])
    return np.array(result)



def write_mission(content):
    content = str(content)
    f = open("mission.txt", "a")
    f.write(content)
    f.write(" \n")
    f.close()

def clear_mission():
    f = open("mission.txt", "w")
    f.write("")
    f.close()

def read_mission():
    mission = []
    with open("mission.txt") as f:
        lines = f.readlines()

    for line in lines:
        mission.append(line.split(" "))
    return mission

def array_to_real_positions(coords, max_height, origin=[0,0], scale=[1,1]):
    for pos in coords:
        #reverse array row indices
        pos[1] = max_height - pos[1] 

        #shift origin
        pos[0] = pos[0] - origin[0] 
        pos[1] = pos[1] - origin[1]

        #center cells
        pos[0] += 0.5
        pos[1] += 0.5

        #scale
        pos[0] *= scale[0]
        pos[1] *= scale[1]

def taxicab_dist(pos1, pos2):
   return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])
        
# function to limit velocity while keeping the same direction



# mission = read_mission()

# for _ in range(len(mission)):
#     print(mission[0])
#     del mission[0]

# print(mission)

# coordinates = formation_coordinates(1, 3, height = 1, displacement=np.array([0,0,0]) ,rotation_angle=0)

# test = []

# for i in range(36):

#     test.append(rotate_coordinates(coordinates, i*10))
# show_coordinates(test, dimension=2)

