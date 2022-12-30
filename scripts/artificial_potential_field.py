#!/usr/bin/env python3

import rospy
import time
import numpy as np
import math

from munkres import Munkres
from utils import *
from geometry_msgs.msg import PoseStamped, Twist
from threading import Thread



class ArtificialPotentialField():
    def __init__(self):

        rospy.init_node("artificial_potential_field")

        self.agent_positions = {}
        self.agents = rospy.get_param("/crazyflies") # Gets params from crazyflies parameters
        self.agent_ids = []
        self.get_agent_ids() # It is also possible to explicitly give id list without using this method
        self.vel_publishers = {}
        self.vel_commands = {}
        self.obstacles = {}
        self.repulsive_pts = {}
        self.rate = rospy.Rate(100) # 10 Hz
        self.stop_velocity = Twist()

        # Gets params from algorithm_params.yaml
        self.num_of_drones = len(self.agents)
        self.attractive_constant = rospy.get_param("/artificial_potential_field/attractive_constant")
        self.repulsive_constant = rospy.get_param("/artificial_potential_field/repulsive_constant")
        self.error_radius = rospy.get_param("/artificial_potential_field/error_radius")
        self.speed_limit = rospy.get_param("/artificial_potential_field/speed_limit")
        self.repulsive_threshold = rospy.get_param("/artificial_potential_field/repulsive_threshold")
        self.potential_field_timeout = rospy.get_param("/artificial_potential_field/potential_field_timeout")

        for id in self.agent_ids:
            vel_command = Twist()

            rospy.Subscriber("/{}/position".format(id), PoseStamped, self.position_callback, callback_args=id)
            self.vel_publishers[id] = rospy.Publisher("/{}/vel_commander".format(id), Twist, queue_size=100)
           
            vel_command.linear.x = 0.0
            vel_command.linear.y = 0.0
            vel_command.linear.z = 0.0
           
            self.vel_commands[id] = vel_command

        Thread(target=rospy.spin).start()
        Thread(target=self.vel_commander_loop).start()
        time.sleep(2) # Wait until all drones are ready


    def limit_velocity(self, velocity_twist, max_velocity):
        velocity = [velocity_twist.linear.x, velocity_twist.linear.y, velocity_twist.linear.z]
        magnitude = math.sqrt(velocity[0]**2 + velocity[1]**2 + velocity[2]**2)
        
        if magnitude > max_velocity:

            unit_vector = np.array([velocity[0]/magnitude, velocity[1]/magnitude, velocity[2]/magnitude])
            
            vel = np.dot(unit_vector, max_velocity)

            result_vel = Twist()
            result_vel.linear.x = vel[0]
            result_vel.linear.y = vel[1]
            result_vel.linear.z = vel[2]

            return result_vel
        else:

            return velocity_twist

    def get_agent_ids(self):
        for agent in self.agents:
            self.agent_ids.append(agent["id"])
        return self.agent_ids

    def position_callback(self, data, id):
        self.agent_positions[id] = [data.pose.position.x, data.pose.position.y, data.pose.position.z]

    #function to publish Twist message to each drone
    def send_vel_commands(self):

        for id in self.agent_ids:
            # print(self.vel_commands[id])
            self.vel_publishers[id].publish(self.limit_velocity(self.vel_commands[id], self.speed_limit))

    def vel_commander_loop(self):
        while not rospy.is_shutdown():
            self.send_vel_commands()
            self.rate.sleep()

    def obstacle_creator_without_drones(self, array_of_obstacles, obstacle_radius = 0.1):
        for i in range(len(array_of_obstacles)): 
            self.obstacles[i] = array_of_obstacles[i] 
            np.append(self.obstacles[i], obstacle_radius) #add radius to the end of the array
        

    def sort_coordinates(self, coordinates):

        sorted_coordinates = [[0, 0, 0]] * self.num_of_drones


        if type(coordinates) == dict:

            cost_matrix = [[math.sqrt((target[0] - self.agent_positions[id][0])**2 
            + (target[1] - self.agent_positions[id][1])**2 
            + (target[2] - self.agent_positions[id][2])**2 ) for target in coordinates.values()] for id in self.agent_ids] # cost matrix is a matrix of distances between each drone and each target
        else:

            cost_matrix = [[math.sqrt((target[0] - self.agent_positions[id][0])**2 
            + (target[1] - self.agent_positions[id][1])**2 
            + (target[2] - self.agent_positions[id][2])**2 ) for target in coordinates] for id in self.agent_ids] # cost matrix is a matrix of distances between each drone and each target

        assigner = Munkres() # Hungarian algorithm
        assignments = assigner.compute(cost_matrix) 
        # assignments is a list of tuples (i, j), where i is the index of the drone and j is the index of the target

        if type(coordinates) == dict:
            for assignment in assignments:
                
                sorted_coordinates[assignment[0]] = list(coordinates.values())[assignment[1]]
                
        else:
            for assignment in assignments:
                sorted_coordinates[assignment[0]] = coordinates[assignment[1]]

        coordinates_by_ids = {}
        for i in range(len(self.agent_ids)):
            coordinates_by_ids[self.agent_ids[i]] = sorted_coordinates[i]

        return coordinates_by_ids

    def is_goal_reached(self, id, goal):# 15 cm default error radius, goal is a numpy array
        
        pose = self.agent_positions[id]

        distance =( (pose[0]-goal[0])**2 + (pose[1]-goal[1])**2 + (pose[2]-goal[2])**2 )**(1/2)
        if distance <= self.error_radius:
            return True
        else:
            return False
    
    def is_formed(self, goals):
        reached =0
        
        for i in self.agent_ids:
             
            if self.is_goal_reached(i, goals[i]):
                reached += 1

        if reached == self.num_of_drones:
            return True
        else:
            return False

    def formation_coordinates(self,distance_between, num_of_edges, height = 1, displacement=np.array([0,0,0]) ,rotation_angle=0):
        vectors = np.zeros(shape=(num_of_edges,3)) #initialize array of vectors
        self.num_of_edges = num_of_edges #number of edges in the polygon
        radius = distance_to_radius(distance_between, num_of_edges) #radius of the polygon
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

    def attractive_force(self, id, target_pose):

        attractive_force_x = (target_pose[0] - self.agent_positions[id][0])*self.attractive_constant
        attractive_force_y = (target_pose[1] - self.agent_positions[id][1])*self.attractive_constant
        attractive_force_z = (target_pose[2] - self.agent_positions[id][2])*self.attractive_constant

        return [attractive_force_x, attractive_force_y, attractive_force_z]
    
    def repulsive_force(self,id): # repuslsive constant must be negative
        repulsive_force_x = 0
        repulsive_force_y = 0
        repulsive_force_z = 0

        for i in self.agent_ids:
                if i == id:
                    continue

                z_distance = self.agent_positions[id][2] - self.agent_positions[i][2]
                y_distance = self.agent_positions[id][1] - self.agent_positions[i][1]
                x_distance = self.agent_positions[id][0] - self.agent_positions[i][0]

                    

                d = ((y_distance**2) + (x_distance**2) + (z_distance**2))**(1/2)


                if d < self.repulsive_threshold:
                    if z_distance != 0:
                        repulsive_force_z += (1/(z_distance**2))*(1/self.repulsive_threshold - 1/z_distance)*self.repulsive_constant * (-(z_distance) / abs(z_distance))
                    if y_distance != 0:
                        repulsive_force_y += (1/(y_distance**2))*(1/self.repulsive_threshold - 1/y_distance)*self.repulsive_constant * (-(y_distance) / abs(y_distance))
                    if x_distance != 0:
                        repulsive_force_x += (1/(x_distance**2))*(1/self.repulsive_threshold - 1/x_distance)*self.repulsive_constant * (-(x_distance) / abs(x_distance))

        for i in range(len(self.obstacles)):
                if i == id:
                    continue
                z_distance = self.agent_positions[id][2] - self.obstacles[i][2]
                y_distance = self.agent_positions[id][1] - self.obstacles[i][1]
                x_distance = self.agent_positions[id][0] - self.obstacles[i][0]

                z_distance = (z_distance - self.obstacles[i]) if (z_distance > 0 )else (z_distance + self.obstacles[i])
                y_distance = (y_distance - self.obstacles[i]) if (y_distance > 0 )else (y_distance + self.obstacles[i])
                x_distance = (x_distance - self.obstacles[i]) if (x_distance > 0 )else (x_distance + self.obstacles[i]) 

            
                if z_distance != 0 and abs(z_distance) < self.repulsive_threshold:
                    repulsive_force_z += (1/(z_distance**2))*(1/self.repulsive_threshold - 1/abs(z_distance))*self.repulsive_constant * (-(z_distance) / abs(z_distance))
                if y_distance != 0 and abs(y_distance) < self.repulsive_threshold:
                    repulsive_force_y += (1/(y_distance**2))*(1/self.repulsive_threshold - 1/abs(y_distance))*self.repulsive_constant * (-(y_distance) / abs(y_distance))
                if x_distance != 0 and abs(x_distance) < self.repulsive_threshold:
                    repulsive_force_x += (1/(x_distance**2))*(1/self.repulsive_threshold - 1/abs(x_distance))*self.repulsive_constant * (-(x_distance) / abs(x_distance))

        for key in self.repulsive_pts:
            z_distance = self.agent_positions[id][2] - self.repulsive_pts[key][2]
            y_distance = self.agent_positions[id][1] - self.repulsive_pts[key][1]
            x_distance = self.agent_positions[id][0] - self.repulsive_pts[key][0]
            d = ((y_distance**2) + (x_distance**2) + (z_distance**2))**(1/2)

            if d < self.repulsive_threshold:
                if z_distance != 0:
                    repulsive_force_z += (1/(z_distance**2))*(1/self.repulsive_threshold - 1/z_distance)*self.repulsive_constant * (-(z_distance) / abs(z_distance))
                if y_distance != 0:
                    repulsive_force_y += (1/(y_distance**2))*(1/self.repulsive_threshold - 1/y_distance)*self.repulsive_constant * (-(y_distance) / abs(y_distance))
                if x_distance != 0:
                    repulsive_force_x += (1/(x_distance**2))*(1/self.repulsive_threshold - 1/x_distance)*self.repulsive_constant * (-(x_distance) / abs(x_distance))

        return [repulsive_force_x, repulsive_force_y, repulsive_force_z]

    def single_potential_field(self, id, coordinates):

        attractive_force_x, attractive_force_y, attractive_force_z = self.attractive_force(target_pose=coordinates, id=id)
        repulsive_force_x, repulsive_force_y, repulsive_force_z = self.repulsive_force(id=id)

        vel_command = Twist()

        vel_command.linear.x = attractive_force_x + repulsive_force_x
        vel_command.linear.y = attractive_force_y + repulsive_force_y
        vel_command.linear.z = attractive_force_z + repulsive_force_z

        self.vel_commands[id] = vel_command

        self.rate.sleep()

    def form_via_potential_field(self, radius, displacement=(0,0,0)): # uses potential field algorithm to form
        self.radius = radius

        coordinates = self.formation_coordinates(radius, self.num_of_drones ,displacement=np.array(displacement))
        coordinates = self.sort_coordinates(coordinates)

        before_starting = time.localtime()

        while not self.is_formed(coordinates):


            if time.mktime(time.localtime()) - time.mktime(before_starting) > self.potential_field_timeout:
                print("TIMEOUT!!!")
                break

            reached = []
            for i in self.agent_ids:
                
                if self.is_goal_reached(i,coordinates[i]):
                    reached.append(i)
                    self.vel_commands[i] = self.stop_velocity
                    continue
                if i in reached: continue


                self.single_potential_field(i, coordinates[i])   

        self.stop_all()   
        rospy.loginfo("Formation complete")

    def stop_all(self):
        for id in self.agent_ids:
            self.vel_commands[id] = self.stop_velocity
        self.rate.sleep() 

    
    def form_polygon(self, distance_between, num_of_edges,height=1, displacement=np.array([0,0,0])): # uses potential field algorithm to form
        
        radius = distance_to_radius(distance_between,num_of_edges) # converts distance between agents to formation radius
        coordinates = self.sort_coordinates(self.formation_coordinates(radius, num_of_edges, height, displacement))

        before_starting = time.localtime()

        
        while not self.is_formed(coordinates):

            if time.mktime(time.localtime()) - time.mktime(before_starting) > self.potential_field_timeout:
                    print("TIMEOUT!!!")
                    break

            for i in self.agent_ids:
                
                self.single_potential_field( i, coordinates[i])
                
        self.stop_all()

    def form_coordinates(self, coordinates): # uses potential field algorithm to form
        coordinates = self.sort_coordinates(coordinates)

        
        before_starting = time.localtime()

        while not self.is_formed(coordinates):

            if time.mktime(time.localtime()) - time.mktime(before_starting) > self.potential_field_timeout:
                    print("TIMEOUT!!!")
                    break

            for i in self.agent_ids:
                
                self.single_potential_field(i, coordinates[i])
                
        self.stop_all()    

    def go(self, vector):
        coordinates = np.zeros(shape=(self.num_of_drones, 3)) # creates array of zeros of shape (num_of_agents,3)

        for i in range(self.num_of_drones): # iterates through agents and adds their positions to the array
            coordinates[i] = self.agent_positions[self.agent_ids[i]]

        for i in range(self.num_of_drones): 
            try:
                coordinates[i][0] += vector[0] # adds the x component of the vector to the x component of the agent's position
                coordinates[i][1] += vector[1]
                coordinates[i][2] += vector[2]
            except IndexError:
                print(coordinates[i][2])
                print(vector[i][2])

        self.form_coordinates(coordinates)     

    def rotate(self, degree, step= 10, duration = 3):
        
        current_coordinates = []
        for id in self.agent_ids:
            current_coordinates.append(self.agent_positions[id])

        for i in range(step):
            rotated_coordinates = rotate_coordinates(current_coordinates, degree/step*i)
            rospy.sleep(duration/step)
            self.form_coordinates(rotated_coordinates)    

    def form_3d(self, radius, num_edges, h=0.5):
        if num_edges == "prism":

            coordinates = self.sort_coordinates(np.concatenate((self.formation_coordinates(0, 1, height=radius+h)
            , self.formation_coordinates(radius, self.num_of_drones-1, height=h))))

            print(coordinates)
            self.form_coordinates(coordinates=coordinates)
            
        elif num_edges == "cylinder": # Here circle function can be used.
            pass
        else:
            coordinates = self.sort_coordinates(np.concatenate((self.formation_coordinates(distance_between=radius
            , num_of_edges=num_edges, height=1.5), self.formation_coordinates(radius, num_of_edges=num_edges, height=0.5))))
            
            print(coordinates)
            self.form_coordinates(coordinates=coordinates)
    def form_v(self,radius,h=0.5,angle=60,displacement=np.zeros(3),direction=0,num_of_agents=-1):
        #radius is distance between two closest agent
        #angle is the angle between two wings of V
        #direction is the rotated coordinates with respect to head of the V
        if num_of_agents==-1:
            num_of_agents=self.num_of_drones
        angle=degree_to_radian(angle)
        coordinates=np.zeros((num_of_agents,3))
        center_displacement_vector=np.zeros(3)
        coordinates[0]=np.array([displacement[0],displacement[1],h+displacement[2]])
        angle_to_used=angle/2
        second_wing_modifier=num_of_agents//2+1
        if not (num_of_agents%2):#if it is certain that for V form num o agents will be odd, then this whole if block can be omitted
            x=(-(num_of_agents/2)*math.sin(angle_to_used)*radius)
            y=(-(num_of_agents/2)*math.cos(angle_to_used)*radius)
            z=h+displacement[2]
            coordinates[num_of_agents-1][0]=x+displacement[0]
            coordinates[num_of_agents-1][1]=y+displacement[1]
            coordinates[num_of_agents-1][2]=z
            second_wing_modifier-=1
            center_displacement_vector=np.array([x,y,z])
        for i in range((num_of_agents-1)//2):
            x=(-(i+1)*math.sin(angle_to_used)*radius)
            y=(-(i+1)*math.cos(angle_to_used)*radius)
            z=h+displacement[2]
            coordinates[i+1][0]=x+displacement[0]
            coordinates[i+1][1]=y+displacement[1]
            coordinates[i+1][2]=z
            coordinates[i+second_wing_modifier][0]=-x+displacement[0]
            coordinates[i+second_wing_modifier][1]=y+displacement[1]
            coordinates[i+second_wing_modifier][2]=z
            center_displacement_vector[1]+=2*y
            center_displacement_vector[2]+=2*z
        center_displacement_vector[0]/=num_of_agents
        center_displacement_vector[1]/=num_of_agents
        center_displacement_vector[2]/=num_of_agents
        for i in range(num_of_agents):
            coordinates[i]=coordinates[i]-center_displacement_vector
        if direction:
            coordinates=rotate_coordinates(coordinates=coordinates,angle=direction)
        coordinates=self.sort_coordinates(coordinates=coordinates)
        print(coordinates)
        self.form_coordinates(coordinates=coordinates)
    def form_star(self,radius,h=0.5,displacement=np.zeros(3)):
        deg_36=math.pi/5
        deg_54=3*math.pi/10
        deg__18=math.pi/10
        deg_72=2*math.pi/5
        radius_2=radius/(2*math.cos(deg_36))
        displacement=np.array(displacement)
        coordinates=np.zeros((10,3))
        coordinates[0]=np.array([radius*math.cos(deg_54),-radius*math.sin(deg_54),h])+displacement
        coordinates[4]=np.array([-radius*math.cos(deg_54),-radius*math.sin(deg_54),h])+displacement
        coordinates[1]=np.array([radius*math.sin(deg_72),radius*math.cos(deg_72),h])+displacement
        coordinates[3]=np.array([-radius*math.sin(deg_72),radius*math.cos(deg_72),h])+displacement
        coordinates[2]=np.array([0,radius,h])+displacement
        coordinates[5]=np.array([radius_2*math.cos(deg__18),-radius_2*math.sin(deg__18),h])+displacement
        coordinates[9]=np.array([-radius_2*math.cos(deg__18),-radius_2*math.sin(deg__18),h])+displacement
        coordinates[6]=np.array([radius_2*math.sin(deg_36),radius_2*math.cos(deg_36),h])+displacement
        coordinates[8]=np.array([-radius_2*math.sin(deg_36),radius_2*math.cos(deg_36),h])+displacement
        coordinates[7]=np.array([0,-radius_2,h])+displacement
        coordinates=self.sort_coordinates(coordinates=coordinates)
        print(coordinates)
        self.form_coordinates(coordinates=coordinates)
    
    def surround_fire(self, field, agent_count):
        circumference = []
        positions = []
        sorted_positions = []

        width = len(field[0])
        height = len(field)

        real_width = 3.5
        real_height = 3.5

        for i in range(0, width):
            for j in range(0, height):
                if field[j][i]: #only check False items (fire zone)
                    continue
                
                #check neighbours of False items (3x3)
                for ii in range(i - 1, i + 2):
                    for jj in range(j - 1, j + 2):
                        if ii < 0 or ii >= width or jj < 0 or jj >= height:
                            continue
                        
                        if field[jj][ii] and [ii, jj] not in circumference:
                            circumference.append([ii, jj])

        agent_dist = len(circumference) / agent_count #optimum distance between agents
        sorted_positions.append(circumference[0])
        
        print("Minimum distance between agents: ", agent_dist, "\n")

        for _ in range(0, len(circumference)-1):
            min_dist = 99999
            cur_pos = sorted_positions[-1]
            circumference.remove(cur_pos)
            next_pos = None
            
            for p in circumference:
                d = abs(cur_pos[0]-p[0]) + abs(cur_pos[1] - p[1]) #taxicab distance
                if d < min_dist:
                    min_dist = d
                    next_pos = p

            sorted_positions.append(next_pos)
        
        print("Sorted positions: ", sorted_positions, "\n")
        
        positions.append(sorted_positions[0])
        for i in range(1, agent_count):
            lowi = int(i*agent_dist)
            highi = int(i*agent_dist) + 1

            min = sorted_positions[lowi]
            max = sorted_positions[highi]

            k1 = 1 - (agent_dist - int(agent_dist));
            k2 = 1 - k1;

            x = min[0]*k1 + max[0]*k2
            y = min[1]*k1 + max[1]*k2
            positions.append([x, y]);

        array_to_real_positions(positions, height, origin=[width/2, height/2], scale=[real_width / width, real_height / height])
        positions = self.sort_coordinates(positions)
        self.form_coordinates(positions)
