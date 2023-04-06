#!/usr/bin/env python3
from socket import timeout
import numpy as np
import math
import time
from Iris import Iris
from threading import Thread
import copy
from munkres import Munkres
from datetime import datetime
import csv
import random

from TurtleBot import TurtleBot
from pycrazyswarm import Crazyswarm
from utils import *
from geometry_msgs.msg import PoseStamped
import copy
import rospy

import rospy
from custom_msg.msg import general_parameters #import custom_msg which is in the workspace

class Swarm:
    def __init__(self,num_of_drones,vehicle, first_time = True, crazyswarm_class=None):

        rospy.init_node("swarm", anonymous=True)

        self.agents = []
        self.vehicle = vehicle
        self.radius = 2
        self.num_of_edges = 3
        self.obstacles = {}
        self.repulsive_pts = {}
        self.isPublishing = False

        self.log = {
    
            "speed": np.array_str(np.array([0,0,0])),
            "time" : datetime.now().strftime("%H:%M:%S:%f"),
            "position" : np.array_str(np.array([0,0,0])),
            "id" : str(0)
        }

        self.logs = {}

        if vehicle == "Crazyflie":
            if first_time:
                self.crazyswarm = Crazyswarm()
                self.agents = self.crazyswarm.allcfs.crazyflies[0:num_of_drones]
                self.agentsById = self.crazyswarm.allcfs.crazyfliesById
                self.timeHelper = self.crazyswarm.timeHelper

                for i in range(len(self.agents)):
                    self.takeoff(i)
            else:
                self.timeHelper = crazyswarm_class.timeHelper
            
            self.timeHelper.sleep(1)
        
        elif vehicle == "Iris":
            
            for i in range(num_of_drones):
                self.add_drone(i)

            for i in range(len(self.agents)):
                pose = self.distance_to_pose(drone_a=self.agents[i], lat_b=simulation_origin_latitude, long_b=simulation_origin_longitude)
                self.agents[i].set_starting_pose(pose[0], pose[1]) #Agent must be in the starting position (locally 0, 0) height does not matter
                self.agents[i].position()

        elif vehicle == "TurtleBot":

            for i in range(num_of_drones):
                self.add_turtlebot(i)

        self.num_of_agents = num_of_drones

        simulation_origin_latitude = 47.3977419
        simulation_origin_longitude = 8.5455935

    def hover(self,sleep_time:float):
        now = self.timeHelper.time()
        while (self.timeHelper.time()-now) < sleep_time:
            # print(self.timeHelper.time())
            self.timeHelper.sleep(0.001)
            for uav in self.agents:
                uav.cmdVelocityWorld(np.array([0.0, 0.0, 0.0]), 0.0)

    def form_3d(self, radius, num_edges, h=0.5):
        if num_edges == "prism":

            coordinates = self.sort_coordinates(np.concatenate((self.formation_coordinates(0, 1, height=radius+h), self.formation_coordinates(radius, len(self.agents)-1, height=h))))
            print(coordinates)
            self.form_coordinates(coordinates=coordinates)
            
        elif num_edges == "cylinder": # Here circle function can be used.
            pass
        else:
            coordinates = self.sort_coordinates(np.concatenate((self.formation_coordinates(distance_between=radius, num_of_edges=num_edges, height=1.5), self.formation_coordinates(radius, num_of_edges=num_edges, height=0.5))))
            print(coordinates)
            self.form_coordinates(coordinates=coordinates)

    def log_to_csv(self):
        for i in self.logs:

            try:

                with open("./agent{}_logs.csv".format(i),"w") as f:
                    writer_csv = csv.writer(f)
                    header = self.log.keys()
                    header_str = []

                    for j in header:
                        j = '{0:''<10}'.format(j)
                        header_str.append(j)

                    #print(header_str)
                    writer_csv.writerow(header_str)
                    for j in self.logs[i]:

                        writer_csv.writerow(j.values())
                    print("LOGGING COMPLETED !!!")
            except LookupError:
                print("FILE COULD NOT OPENED")

    def add_log(self, speed, time, position, id):
        self.log["speed"] = speed
        self.log["time"] = time
        self.log["position"] = position
        self.log["id"] = id

        try:
            self.logs[str(id)].append(copy.copy(self.log))
        except KeyError:
            self.logs[str(id)] = []
            self.logs[str(id)].append(copy.copy(self.log))

    def delta_angle(self, x,y):
        angle = math.atan(y/x)

        if x < 0 and y >= 0:
            angle =  angle + math.pi
        elif x < 0 and y < 0:
            angle =  angle + math.pi
        else:
            angle =  angle + math.pi

        return angle if math.pi <=angle else -angle

    def is_goal_reached(self, id, goal,error_radius=0.25):# 15 cm default error radius, goal is a numpy array
        
        pose = self.agents[id].position()


        distance =( (pose[0]-goal[0])**2 + (pose[1]-goal[1])**2 + (pose[2]-goal[2])**2 )**(1/2)
        if distance <= error_radius:
            return True
        else:
            #print(goal[0]-pose[0]," ",id)
            return False
    
    def is_formed(self, goals):
        reached =0
        for i in range(self.num_of_agents):
             
            if self.is_goal_reached(i, goals[i]):
                reached += 1

        if reached == self.num_of_agents:
            return True
        else:
            return False


    def formation_coordinates(self,distance_between, num_of_edges, height = 1, displacement=np.array([0,0,0]) ,rotation_angle=0):
        vectors = np.zeros(shape=(num_of_edges,3)) # [id][0: for x, 1: for y, 2: for z]
        self.num_of_edges = num_of_edges
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

    def sort_coordinates(self, coordinates):
        sorted_coordinates = [[0, 0]]*self.num_of_agents
        cost_matrix = [[math.sqrt((target[0] - drone.position()[0])**2 + (target[1] - drone.position()[1])**2 ) for target in coordinates] for drone in self.agents]
        
        assigner = Munkres()
        assignments = assigner.compute(cost_matrix)
        
        for assignment in assignments:
            sorted_coordinates[assignment[0]] = coordinates[assignment[1]]

        return sorted_coordinates

    def attractive_force(self, id, target_pose, attractive_constant = 2):
        speed_limit = 0.9 # must be float

        attractive_force_x = (target_pose[0] - self.agents[id].position()[0])*attractive_constant
        attractive_force_y = (target_pose[1] - self.agents[id].position()[1])*attractive_constant
        attractive_force_z = (target_pose[2] - self.agents[id].position()[2])*attractive_constant

        return min(max(float(attractive_force_x),-1*speed_limit),speed_limit), min(max(float(attractive_force_y),-1*speed_limit),speed_limit), min(max(float(attractive_force_z),-1*speed_limit),speed_limit)

    
    def repulsive_force(self,id, repulsive_constant =-0.9,repulsive_threshold = 1.3): # repuslsive constant must be negative
        repulsive_force_x = 0
        repulsive_force_y = 0
        repulsive_force_z = 0
        speed_limit = 0.5 # must be float

        for i in range(len(self.agents)):
                if i == id:
                    continue
                z_distance = self.agents[id].position()[2] - self.agents[i].position()[2]
                y_distance = self.agents[id].position()[1] - self.agents[i].position()[1]
                x_distance = self.agents[id].position()[0] - self.agents[i].position()[0]

                d = ((y_distance**2) + (x_distance**2) + (z_distance**2))**(1/2)


                if d < repulsive_threshold:
                    if z_distance != 0:
                        repulsive_force_z += (1/(z_distance**2))*(1/repulsive_threshold - 1/z_distance)*repulsive_constant * (-(z_distance) / abs(z_distance))
                    if y_distance != 0:
                        repulsive_force_y += (1/(y_distance**2))*(1/repulsive_threshold - 1/y_distance)*repulsive_constant * (-(y_distance) / abs(y_distance))
                    if x_distance != 0:
                        repulsive_force_x += (1/(x_distance**2))*(1/repulsive_threshold - 1/x_distance)*repulsive_constant * (-(x_distance) / abs(x_distance))

        for i in range(len(self.obstacles)):
                if i == id:
                    continue
                z_distance = self.agents[id].position()[2] - self.obstacles[i][2]
                y_distance = self.agents[id].position()[1] - self.obstacles[i][1]
                x_distance = self.agents[id].position()[0] - self.obstacles[i][0]

                z_distance = (z_distance - self.obstacles[i]) if (z_distance > 0 )else (z_distance + self.obstacles[i])
                y_distance = (y_distance - self.obstacles[i]) if (y_distance > 0 )else (y_distance + self.obstacles[i])
                x_distance = (x_distance - self.obstacles[i]) if (x_distance > 0 )else (x_distance + self.obstacles[i]) 

            
                if z_distance != 0 and abs(z_distance) < repulsive_threshold:
                    repulsive_force_z += (1/(z_distance**2))*(1/repulsive_threshold - 1/abs(z_distance))*repulsive_constant * (-(z_distance) / abs(z_distance))
                if y_distance != 0 and abs(y_distance) < repulsive_threshold:
                    repulsive_force_y += (1/(y_distance**2))*(1/repulsive_threshold - 1/abs(y_distance))*repulsive_constant * (-(y_distance) / abs(y_distance))
                if x_distance != 0 and abs(x_distance) < repulsive_threshold:
                    repulsive_force_x += (1/(x_distance**2))*(1/repulsive_threshold - 1/abs(x_distance))*repulsive_constant * (-(x_distance) / abs(x_distance))

        for key in self.repulsive_pts:
            z_distance = self.agents[id].position()[2] - self.repulsive_pts[key][2]
            y_distance = self.agents[id].position()[1] - self.repulsive_pts[key][1]
            x_distance = self.agents[id].position()[0] - self.repulsive_pts[key][0]
            d = ((y_distance**2) + (x_distance**2) + (z_distance**2))**(1/2)

            if d < repulsive_threshold:
                if z_distance != 0:
                    repulsive_force_z += (1/(z_distance**2))*(1/repulsive_threshold - 1/z_distance)*repulsive_constant * (-(z_distance) / abs(z_distance))
                if y_distance != 0:
                    repulsive_force_y += (1/(y_distance**2))*(1/repulsive_threshold - 1/y_distance)*repulsive_constant * (-(y_distance) / abs(y_distance))
                if x_distance != 0:
                    repulsive_force_x += (1/(x_distance**2))*(1/repulsive_threshold - 1/x_distance)*repulsive_constant * (-(x_distance) / abs(x_distance))


        return min(max(float(repulsive_force_x),-1*speed_limit),speed_limit), min(max(float(repulsive_force_y),-1*speed_limit),speed_limit), min(max(float(repulsive_force_z),-1*speed_limit),speed_limit)


    def single_potential_field(self, id, coordinates):

        #for _ in range(4000):
        #print("id: {}  pose: {}".format(id, self.agents[id].position()))

        attractive_force_x, attractive_force_y, attractive_force_z = self.attractive_force(target_pose=coordinates[id], id=id, attractive_constant = 5)
        repulsive_force_x, repulsive_force_y, repulsive_force_z = self.repulsive_force(id=id)

        vel_x = attractive_force_x + repulsive_force_x
        vel_y = attractive_force_y + repulsive_force_y
        vel_z = attractive_force_z + repulsive_force_z

        turtleBot_constant = 0.1
        rotational_constant = 0.3

        if self.vehicle == "Crazyflie":
            self.agents[id].cmdVelocityWorld(np.array([vel_x, vel_y, vel_z]), yawRate=0)
            self.timeHelper.sleep(0.001)
            self.add_log("{}, {}, {}, ".format(vel_x, vel_y, 0), datetime.now(),"{}, {}, {}, ".format(self.agents[id].position()[0],self.agents[id].position()[1],self.agents[id].position()[2]),id)

        elif self.vehicle == "TurtleBot": 
            
            angle_of_agent = self.agents[id].orientation() 
            vector_angle = angle_of_vector(vel_x, vel_y)
            dA = vector_angle - angle_of_agent   
            linear_vel = vel_x*vel_y* math.cos(dA)      
            #print("vel:  {}".format(np.array([linear_vel * turtleBot_constant, 0, 0])))
            self.agents[id].cmdVelocityWorld(np.array([linear_vel * turtleBot_constant, 0, 0]), yawRate=dA * rotational_constant)
            self.add_log(np.array([linear_vel * turtleBot_constant, 0, 0]), datetime.now(),self.agents[id].position(),id)
            time.sleep(0.01)
            #print("velY: {} velX: {}\ncurrA: {} targetA: {}".format(vel_y,vel_x,angle_of_agent, angle_of_vector))

        if self.vehicle == "Iris":
            self.agents[id].move_global(coordinates[id][0], coordinates[id][1], 5) # makes more stable

    

    def form_via_potential_field(self, radius, timeout = 10, displacement=(0,0,0)): # uses potential field algorithm to form
        self.radius = radius

        coordinates = self.formation_coordinates(radius, self.num_of_agents,displacement=np.array(displacement))
        coordinates = self.sort_coordinates(coordinates)
        if self.vehicle == "Crazyflie" or self.vehicle == "TurtleBot":
            before_starting = time.localtime()

            while not self.is_formed(coordinates):
                if self.isPublishing:
                    self.pub_pose()

                if time.mktime(time.localtime()) - time.mktime(before_starting) > timeout:
                    print("TIMEOUT!!!")
                    break

                reached = []
                for i in range(len(self.agents)):
                    
                    if self.is_goal_reached(i,coordinates[i]):
                        reached.append(i)
                        self.agents[i].cmdVelocityWorld(np.array([0.0, 0.0, 0.0]), 0.0)
                        continue
                    if i in reached: continue

                    self.single_potential_field(i, coordinates)   
            self.stop_all()                 
                
            
        elif self.vehicle == "Iris":
            for i in range(len(self.agents)):
                    #Thread(target=self.single_potential_field, args = (radius,i)).start()
                    self.single_potential_field(i, coordinates)
                    print("call")

            self.stop_all()
            self.timeHelper.sleep(4)  
                  
    def form_polygon(self, distance_between, num_of_edges,height=1, displacement=np.array([0,0,0]), timeout = 10): # uses potential field algorithm to form
        
        radius = distance_to_radius(distance_between,num_of_edges) # converts distance between agents to formation radius
        coordinates = self.sort_coordinates(self.formation_coordinates(radius, num_of_edges, height, displacement))

        before_starting = time.localtime()

        if self.vehicle == "Crazyflie":
            while not self.is_formed(coordinates):

                if time.mktime(time.localtime()) - time.mktime(before_starting) > timeout:
                        print("TIMEOUT!!!")
                        break
            #for i in range(4000):
                for i in range(len(self.agents)):
                    
                    self.single_potential_field( i, coordinates)
                    
            self.stop_all()
            self.timeHelper.sleep(4)           

    def form_coordinates(self, coordinates, timeout = 10): # uses potential field algorithm to form
        coordinates = self.sort_coordinates(coordinates)

        if self.vehicle == "Crazyflie":
            before_starting = time.localtime()

            while not self.is_formed(coordinates):

                if time.mktime(time.localtime()) - time.mktime(before_starting) > timeout:
                        print("TIMEOUT!!!")
                        break

                for i in range(len(self.agents)):
                    
                    self.single_potential_field(i, coordinates)
                    
            self.stop_all()
                  
    def split_formation(self):
        swarm1 = Swarm(self.num_of_agents//2, self.vehicle, False, self.crazyswarm)
        swarm1.agents = self.crazyswarm.allcfs.crazyflies[0 : self.num_of_agents//2]
        swarm2 = Swarm(self.num_of_agents-self.num_of_agents//2, self.vehicle, False, self.crazyswarm)
        swarm2.agents = self.crazyswarm.allcfs.crazyflies[self.num_of_agents//2 : self.num_of_agents]

        swarm1.go((-2,-2,1))
        swarm2.go((2,2,1))

        swarm1.form_polygon(3, 3, 1, np.array([-2,-2,0]))
        swarm2.form_polygon(3, 3, 1, np.array([2,2,0]))
    
        return swarm1, swarm2

    def obstacle_creator(self, num_obstacles, obstacle_radius):
        for i in range(num_obstacles):
            self.obstacles[i] = self.agents[i].position()
            # self.obstacles[i].append(0.1)
            np.append(self.obstacles[i], obstacle_radius)
        self.agents = self.agents[num_obstacles:self.num_of_agents]
        self.num_of_agents = self.num_of_agents-num_obstacles
    
    def obstacle_creator_without_drones(self, array_of_obstacles, obstacle_radius = 0.1):
        for i in range(len(array_of_obstacles)):
            self.obstacles[i] = array_of_obstacles[i]
            # self.obstacles[i].append(obstacle_radius)
            np.append(self.obstacles[i], obstacle_radius)


    def add_drone(self,id):
   
        self.agents.append(Iris(id))

    def add_turtlebot(self,id):
   
        self.agents.append(TurtleBot(id))

    def swarm_square(self, length):
        if self.vehicle == "Crazyflie":

            self.go(np.array([length,0,0]))
            self.go(np.array([0,length,0]))
            self.go(np.array([-length,0,0]))
            self.go(np.array([0,-length,0]))
        else:

            for i in range(len(self.agents)):
                Thread(target = self.agents[i].draw_square, args=(5,)).start()

    def distance_of_drones(self,drone_a, drone_b): # in meters

        lat_b = drone_b.gps_pose_getter().latitude
        long_b = drone_b.gps_pose_getter().longitude

        return self.distance_to_pose(drone_a, lat_b, long_b)

    def distance_to_pose(self, drone_a, lat_b, long_b): # in meters

        lat_a = drone_a.gps_pose_getter().latitude
        x_distance = 111139 * (lat_a - lat_b)
        long_a = drone_a.gps_pose_getter().longitude
        R = 63678.137

        dlat = lat_b * math.pi / 180 - lat_a * math.pi/180 
        dlon = long_b * math.pi / 180 - long_a * math.pi/180 
        a = math.sin(dlat/2) * math.sin(dlat/2) + math.cos(lat_a*math.pi / 180) * math.cos(lat_b * math.pi / 180) * math.sin(dlon/2) * math.sin(dlon/2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        d = R * c
        abs_distance = d * 100

        y_distance = math.sqrt(abs_distance**2-x_distance**2)

        x_coefficient = 1
        y_coefficient = 1

        if(lat_a < lat_b):
            x_coefficient = -1
        if(long_a < long_b):
            y_coefficient = -1

        return x_distance*x_coefficient, y_distance*y_coefficient, abs_distance

    def print_drones_pose(self):
        j = 0
        for i in self.agents:
            print(j)
            i.position()
            j += 1
        print("------------")
        time.sleep(1)

    def return_starting_pose(self):
        for i in self.agents:
            Thread(target=i.move_local, args=(0,0,5)).start()

    def takeoff(self, id, height=1):
        if self.vehicle == "TurtleBot":
            print("TURTLES CAN NOT FLY !!!")
            return
        elif self.vehicle == "Crazyflie":
            self.agents[id].takeoff(targetHeight=height, duration=1)
        
    def hover(self,sleep_time:float):
        now = self.timeHelper.time()
        while (self.timeHelper.time()-now) < sleep_time:
            # print(self.timeHelper.time())
            self.timeHelper.sleep(0.001)
            for uav in self.agents:
                uav.cmdVelocityWorld(np.array([0.0, 0.0, 0.0]), 0.0)
            if self.isPublishing:
                self.pub_pose()

    def land(self):
        if self.vehicle == "Crazyflie":
            for i in range(len(self.agents)):
                self.agents[i].land(0.03, 2)

    def stop_all(self):
        if self.vehicle == "Iris" or self.vehicle == "TurtleBot":
            for i in range(len(self.agents)):
                self.agents[i].velocity_command()
        if self.vehicle == "Crazyflie":
            for i in range(len(self.agents)):
                self.agents[i].cmdVelocityWorld(np.array([0.0, 0.0, 0.0]), yawRate=0)

    def add_agent_to_formation(self):
        self.agents.append(self.crazyswarm.allcfs.crazyflies[len(self.agents)])
        self.num_of_agents += 1

        self.takeoff(len(self.agents)-1)
        self.timeHelper.sleep(1)
        self.form_polygon(self.radius, len(self.agents))
        self.timeHelper.sleep(2)

    def omit_agent(self):
        self.agents[len(self.agents)-1].land(0.03, 2)
        self.num_of_agents -= 1
        self.agents.pop()
        self.timeHelper.sleep(1)
        self.form_via_potential_field(self.radius)
        self.timeHelper.sleep(2)

    def omit_agent_by_id(self, id):
        self.agents[id].land(0.03, 2)
        self.num_of_agents -= 1
        del self.agents[id]
        self.timeHelper.sleep(1)
        self.form_via_potential_field(self.radius)
        self.timeHelper.sleep(2)

    def go(self, vector):
        coordinates = np.zeros(shape=(len(self.agents),3))

        for i in range(len(self.agents)):
            coordinates[i] = self.agents[i].position()

        for i in range(len(self.agents)):
            try:
                coordinates[i][0] += vector[0]
                coordinates[i][1] += vector[1]
                coordinates[i][2] += vector[2]
            except IndexError:
                print(coordinates[i][2])
                print(vector[i][2])

        self.form_coordinates(coordinates)

    def star_formation(self, radius = 4, angle=0, height = 1):
        pentagon1 = self.formation_coordinates(radius,5,height,np.array([0,0,0]),angle)
        pentagon2 = self.formation_coordinates(radius/2,5,height,np.array([0,0,0]),36+angle)
        pentagons = []

        for i in pentagon1:
            pentagons.append(i)
        for i in pentagon2:
            pentagons.append(i)

        self.form_coordinates(pentagons)
        self.timeHelper.sleep(2)

    def form_pyramid(self):
        coordinates = self.formation_coordinates(self.radius, 4)
        coordinates = list(coordinates)
        coordinates.append(np.array([0,0,2]))
        coordinates = self.sort_coordinates(coordinates)

        self.form_coordinates(coordinates)

    def rotate(self, degree, step= 10, duration = 3):
        
        current_coordinates = []
        for agent in self.agents:
            current_coordinates.append(agent.position())

        for i in range(step):
            rotated_coordinates = rotate_coordinates(current_coordinates, degree/step*i)
            self.timeHelper.sleep(duration/step)
            self.form_coordinates(rotated_coordinates, timeout=1)

    
# Publishes the location of agents to a ros topic in order to simulate opponent team in cargo mission.
    def cargo_opponent_simulation(self): 
        pose = PoseStamped()
        poses = []
        
        # rospy.Publisher("cargo", PoseStamped[], )

        for agent in self.agents:
            agent_pose = agent.position()
            pose.pose.position.x = agent_pose[0]
            pose.pose.position.y = agent_pose[1]
            pose.pose.position.z = agent_pose[2]

            poses.append(copy.copy(pose))

        print(poses)
        print("-----------------")
        
    
    def init_pose_pub(self):
        self.pose_publishers = {id:rospy.Publisher(f"general_parameters/METUrone_{id:X}", general_parameters, queue_size=10) for id in self.agentsById}
        self.isPublishing = True

    def pub_pose(self):
        for id in self.agentsById:
            msg = general_parameters()
            msg.pose.x,msg.pose.y,msg.pose.z = self.agentsById[id].position()
            self.pose_publishers[id].publish(msg)

    def land_my(self, id, land_height=0.05):
        gain = 0.5
        d_h = self.agents[id].position()[2] - land_height
        while d_h > 0:
            d_h = self.agents[id].position()[2] - land_height
            self.agents[id].cmdVelocityWorld(np.array([0, 0, -d_h*gain]), yawRate=0)
            self.timeHelper.sleep(0.1)
        self.agents[id].cmdVelocityWorld(np.array([0, 0, 0]), yawRate=0)
        self.agents[id].stop()

    def is_swarm_landed(self, error=0.05):
        result = True
        for agent in self.agents:
            if agent.position()[2] > error:
                result = False

        return result

    def land_swarm(self, error = 0.05):
        gain = 0.5
        before_starting = time.localtime()
        timeout = 10
        
        while not self.is_swarm_landed(error=error):

            if time.mktime(time.localtime()) - time.mktime(before_starting) > timeout:
                    print("TIMEOUT!!!")
                    break

            for agent in self.agents:
                d_h = agent.position()[2] - error
                print(d_h)
                agent.cmdVelocityWorld(np.array([0, 0, -d_h*gain]), yawRate=0)
                self.timeHelper.sleep(0.01)
        
        for agent in self.agents:
            agent.cmdVelocityWorld(np.array([0, 0, 0]), yawRate=0)
            agent.stop()


    def land_prism(self, d_between):
        num_of_agents = len(self.agents)
        print(num_of_agents)
        coordinates1 = formation_coordinates(d_between, num_of_agents//2, height=d_between+0.5)
        angle = 360/(num_of_agents)
        coordinates1_rotated = rotate_coordinates(coordinates1, angle)

        coordinates2 = formation_coordinates(d_between, num_of_agents//2, height=0.5)

        coordinates = self.sort_coordinates(np.concatenate((coordinates1_rotated, coordinates2)))

        self.form_coordinates(coordinates)
        self.land_swarm()
