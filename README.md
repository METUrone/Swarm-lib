# Gazebo Simulation for Swarm-lib

**Install ROS**

from http://wiki.ros.org/noetic/Installation/Ubuntu

**Install Dependencies**

```
sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh  
```
```
sudo apt install python3-pip
pip3 install --user future
pip3 install empy toml numpy packaging jinja2
pip3 install kconfiglib
pip3 install --user jsonschema
pip3 install munkres
pip3 install --user pyros-genmsg
```
```
sudo apt install python3-lxml libxml2-utils
sudo apt install python3-tk
sudo apt install libgstreamer1.0-0 gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly
sudo apt install gstreamer1.0-libav gstreamer1.0-doc gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio
```
```
sudo apt-get install libgeographic-dev
sudo apt-get install geographiclib-tools
sudo apt-get install libgstreamer-plugins-base1.0-dev
```
**Install PX4-Autopilot**

```
git clone --branch release/1.12 https://github.com/PX4/PX4-Autopilot.git --single-branch 
cd PX4-Autopilot
mkdir build && cd build
cmake ..
make
cd ~/PX4-Autopilot
DONT_RUN=1 make px4_sitl_default gazebo
```
```
cd ~

echo "source $(pwd)/PX4-Autopilot/Tools/setup_gazebo.bash $(pwd)/PX4-Autopilot $(pwd)/PX4-Autopilot/build/px4_sitl_default" >> ~/.bashrc
echo "export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/PX4-Autopilot" >> .bashrc
echo "export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/PX4-Autopilot/Tools/sitl_gazebo" >> ~/.bashrc

source ~/.bashrc
```
**Install Swarm-lib (Our Package)**

```
mkdir -p Swarm/src && cd Swarm/src
git clone --branch gazebo https://github.com/METUrone/Swarm-lib.git --single-branch
cd ~/Swarm
catkin_make
```

**Launch the Simulation**
```console
source devel/setup.bash
roslaunch swarm gazebo.launch
```

### Alternative: Install on a Docker Container
1. Build a container

```
git clone https://github.com/PX4/PX4-containers.git
cd PX4-containers/docker
docker build -t px4io/px4-dev-ros-noetic -f Dockerfile_ros-noetic .
```

2. Run the following command to create a container, and run it.
```
echo 'xhost +local:docker' >> ~/.bashrc 
source ~/.bashrc
docker run -it -w /root/ -v ~/catkin_ws:/root/catkin_ws -v /tmp/.X11-unix:/tmp/.X11-unix:ro -e DISPLAY=${DISPLAY} -e --name=px4 px4io/px4-dev-ros-noetic bash
```
**Warning:** This command should be called only once. To run the same container later use the following commands. sudo docker start my-docker-container runs the container that we named my-docker-container. sudo docker exec -it my-docker-container bash opens a terminal where we can run commands in the running container.
```
sudo docker start px4
sudo docker exec -it px4 bash
```

3.  Clone and build
```
git clone --branch release/1.12 https://github.com/PX4/PX4-Autopilot.git --single-branch 
cd PX4-Autopilot
DONT_RUN=1 make px4_sitl_default gazebo
```
4. Source PX4
```
cd ~

echo "source $(pwd)/PX4-Autopilot/Tools/setup_gazebo.bash $(pwd)/PX4-Autopilot $(pwd)/PX4-Autopilot/build/px4_sitl_default" >> ~/.bashrc
echo "export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/PX4-Autopilot" >> .bashrc
echo "export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/PX4-Autopilot/Tools/sitl_gazebo" >> ~/.bashrc

source ~/.bashrc
```
```
pip3 install munkres
```

5. Install Swarm-lib (Our Package)*
```
mkdir -p Swarm/src && cd Swarm/src
git clone --branch gazebo https://github.com/METUrone/Swarm-lib.git --single-branch
cd ~/Swarm
catkin_make
```
6. Launch the Simulation
```console
source devel/setup.bash
roslaunch swarm gazebo.launch
```

# Basic Structure
This package consists of two main parts artificial_potential_field.py and iris_controller.py. artificial_potential_field nodes subscribes to current poses of uav{uav_id}/mavros/local_position/pose topic whose publisher is mavros, and publishes the instant velocity for each agent via {uav_id}/vel_commander topic. Then iris_controller node takes these velocity commands, send them to PX4 controller via /uav0/mavros/setpoint_velocity/cmd_vel_unstamped topic and publishes resulting poses to {uad_id}/position topic whose subscriber is artificial_potential_field node. This cycle continues until all tasks are achieved.

The following graph is a description of such a system with two agents. For the systems with more agents, description consists of the repetition of this structure.


![rosgraph](https://user-images.githubusercontent.com/85796946/188333321-a25cf9a9-79fa-4273-9a87-2b285b6a855b.png)

Iris_takeoff nodes are responsible for keeping drones in flight, and the mission_planner node is an interface where we specify the tasks we want to achieve (A GUI will be placed soon). Since crazyflie ids start from 193, here /uavx/ corresponds to the image of the crazyflie with id x in the simulation.

# Example Run
Here is an example run of a simulation where agents are performing forming formation and obstacle avoidance tasks.



https://drive.google.com/file/d/1jfvP-AinxXbAPjuOdyk524cK3rmOSeiJ/view?usp=sharing

# Comman Issues:

- If you cannot see drones in the gazebo simulation try running these first to kill previously opened gazebo processes:
```
killall gzserver
killall gzclient
```

- If you are getting cc1plus error during building PX4 try running the same build command with the `-j4` flag to limit the cpu usage.

# Code Explanations

The Artificial Potential Field file has one class ArtificialPotentialField and ArtificialPotentialField class has 24 functions:

1-Initializing function(__init__)
Initializes a node in ROS called "artificial_potential_field". Initializes agents according to the parameters from algorithm_params.yaml with velocity 0. Initializes obstacles from params. Then waits for all drones to be ready.

2-limit_velocity
Takes the current velocity(velocity_twist) and max velocity(max_velocity) as parameters. If the current velocity is bigger than max velocity, decreases the current velocity to max velocity while maintaining the direction.

3-get_agent_ids
Returns an array of agent ids.

4-position_callback
Takes position data(data) and id of agent(id) and updates the position of agent with said id to received position data.

5-send_vel_commands
Publishes the Twist velocities to each drone.

6-vel_commander_loop
Publishes Twist velocities to drones continuously while ROS is working.

7-obstacle_creator_without_drones
Takes an array of obstacles(array_of_obstacles) and obstacle radius(obstacle_radius)(default is 0.1) and appends these obstacles to the classes obstacles array and numpy array. 

8-sort_coordinates


9-is_goal_reached
Takes the id of drone(id) and and a numpy array representing goal(goal), and returns whether the drone has reached the goal or not with a 15 cm error radius.

10-is_formed
Takes a numpy array representing goals(goals) and checks for every drone whether the drone has reached the goal or not. If all drones have reached the goal, returns True, else False. 

11-formation_coordinates
Takes number of edges(num_of_edges), distance between drones(distance_between), height(height)(default is 1), a displacement array(displacement)(default is a numpy array of zeroes) and rotation angle(rotation_angle)(defualt is 0) of the formation and returns an array of vectors that will create this formation.

12-attractive_force
Takes id of drone(id) and target position(target_pose) and returns the attractive force vector according to Artificial Potential Field algorithm for the drone.

13-repulsive_force
Takes id of drone(id) and according to Artificial Potential Field algorithm and the array of obstacles, returns repulsive force vector for the drone.

14-single_potential_field
Takes id of one drone(id), finds attractive and repulsive forces of the drone and updates its velocity according to found attractive and repulsive forces. 

15-form_via_potential_field
Takes radius of formation(radius) and a displacement vector(displacement)(default is an array of zeroes) and uses Artificial Potential Field algorithm and formation_coordinates function with default variables to create the formation. When the formation is complete, logs "Formation complete".

16-stop_all
Stops all drones by setting the velocities to zero.

17-form_polygon
Takes radius of polygon formation(radius) and a displacement vector(displacement)(default is an array of zeroes) and uses Artificial Potential Field algorithm and formation_coordinates function with default variables to form the polygon.

18-form_coordinates
Takes an array of coordinates(coordinates) and forms coordinates by Artificial Potential Field algorithm.

19-go
Takes a vector(vector) and moves all drones by this vector.  

20-rotate
Takes degree of rotation(degree), step amount of rotation(step)(default is 10) and duration of the rotation(duration)(default is 3) and rotates all drones around a point according to the variables.

21-form_3d
Takes radius of the formation(radius), string information of formation(num_edges) and height(h). Here num_edges can either be "prism" or "cylinder". Cylinder formation is not initialized yet in this function. For prism formation, it uses form_coordinates function to form a prism formation.

22-form_v
Takes the distance between two closest agents(radius), angle between two wings of V(angle)(default is 60 degrees), height(h)(default is 0.5), the direction of the formation(direction)(default is 0) and number of agents(num_of_agents)(default is -1) and forms a V formation by calculating relevant coordinates and using form_coordinates function.

23-form_star
Takes radius of formation(radius), height(h) and a displacement vector(displacement)(default is an array of zeroes) and forms a star formation by calculating relevant coordinates and using form_coordinates function.

24-surround_fire
Gives drones coordinates to surround the fire according to the fire coordinates obtained via computer vision.


