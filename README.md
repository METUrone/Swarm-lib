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

# Artificial Potential Field Function Explanations

The Artificial Potential Field file has one class ArtificialPotentialField and ArtificialPotentialField class has 24 functions:

**1-Initializing function(__init__)**

Initializes a node in ROS called "artificial_potential_field". Initializes agents according to the parameters from algorithm_params.yaml with velocity 0. Initializes obstacles from params. Then waits for all drones to be ready.

**2-limit_velocity**
| Parameter     |   Explanation |
| ------------- | ------------- |
| velocity_twist | current velocity |
| max_velocity | max velocity |

If the current velocity is bigger than max velocity, decreases the current velocity to max velocity while maintaining the direction.

**3-get_agent_ids**

Returns an array of agent ids.

**4-position_callback**
| Parameter     |   Explanation |
| ------------- | ------------- |
| data | position data |
| id | agent id |

Updates the position of agent with received id to received position data.

**5-send_vel_commands**

Publishes the Twist velocities to each drone.

**6-vel_commander_loop**

Publishes Twist velocities to drones continuously while ROS is working.

**7-obstacle_creator_without_drones**
| Parameter     |   Explanation |
| ------------- | ------------- |
| array_of_obstacles | array of obstacle positions |
| obstacle_radius | radius of obstacles (default is 0.1) |

Appends the received obstacles to the classes obstacles array and numpy array. 

**8-sort_coordinates**


**9-is_goal_reached**
| Parameter     |   Explanation |
| ------------- | ------------- |
| id | id of drone|
| goal | a numpy array representing goal |

Returns whether the drone has reached the goal or not with a 15 cm error radius.

**10-is_formed**
| Parameter     |   Explanation |
| ------------- | ------------- |
| goals | a numpy array representing goals |

Checks for every drone whether the drone has reached the goal or not. If all drones have reached the goal, returns True, else False. 

**11-formation_coordinates**
| Parameter     |   Explanation |
| ------------- | ------------- |
| num_of_edges | number of edges|
| distance_between | distance between drones |
| height | height (default is 1)|
| displacement | a displacement array (default is a numpy array of zeroes)|
|rotation_angle | rotation angle (default is 0)|

Returns an array of vectors that will create this formation.

**12-attractive_force**
| Parameter     |   Explanation |
| ------------- | ------------- |
| id | id of drone|
| target_pose | target position |

Returns the attractive force vector according to Artificial Potential Field algorithm for the drone.

**13-repulsive_force**
| Parameter     |   Explanation |
| ------------- | ------------- |
| id | id of drone|

According to Artificial Potential Field algorithm and the array of obstacles, returns repulsive force vector for the drone.

**14-single_potential_field**
| Parameter     |   Explanation |
| ------------- | ------------- |
| id | id of drone|

Finds attractive and repulsive forces of the drone and updates its velocity according to found attractive and repulsive forces. 

**15-form_via_potential_field**
| Parameter     |   Explanation |
| ------------- | ------------- |
| radius | radius of formation |
| displacement | displacement vector (default is an array of zeroes) |

Uses Artificial Potential Field algorithm and formation_coordinates function with default variables to create the formation. When the formation is complete, logs "Formation complete".

**16-stop_all**

Stops all drones by setting the velocities to zero.

**17-form_polygon**
| Parameter     |   Explanation |
| ------------- | ------------- |
| radius | radius of polygon formation|
| displacement | displacement vector (default is an array of zeroes) |

Uses Artificial Potential Field algorithm and formation_coordinates function with default variables to form the polygon.

**18-form_coordinates**
| Parameter     |   Explanation |
| ------------- | ------------- |
| coordinates | array of coordinates|

Forms coordinates by Artificial Potential Field algorithm.

**19-go**
| Parameter     |   Explanation |
| ------------- | ------------- |
| vector | a vector |

Moves all drones by the vector.  

**20-rotate**
| Parameter     |   Explanation |
| ------------- | ------------- |
| degree | degree of rotation|
| step | step amount of rotation (default is 10)|
| duration | duration of the rotation (default is 3)|

Rotates all drones around a point according to the variables.

**21-form_3d**
| Parameter     |   Explanation |
| ------------- | ------------- |
| radius | radius of the formation|
| num_edges | string information about formation (can either be "prism" or "cylinder")|
| height | height|

Cylinder formation is not initialized yet in this function. For prism formation, it uses form_coordinates function to form a prism formation.

**22-form_v**
| Parameter     |   Explanation |
| ------------- | ------------- |
| radius | the distance between two closest agents|
| angle | angle between two wings of V (default is 60 degrees)|
|h | height (default is 0.5)|
|direction | the direction of the formation |
|num_of_agents | number of agents (default is -1)|

Forms a V formation by calculating relevant coordinates and using form_coordinates function.

**23-form_star**
| Parameter     |   Explanation |
| ------------- | ------------- |
| radius | the distance between two closest agents|
| h | height |
|displacement| displacement vector (default is an array of zeroes)|

Forms a star formation by calculating relevant coordinates and using form_coordinates function.

**24-surround_fire**

Gives drones coordinates to surround the fire according to the fire coordinates obtained via computer vision.

# Yaml Files

There are 2 yaml files in the config folder with spesifications about variables inside them.

**1-algoritm_parameters.yaml**

Has spesifications about variables used in the artificial potential field algorithm.

| Parameter     |   Explanation |
| ------------- | ------------- |
| attractive_constant | attractive constant of the artificial potential field algorithm|
| repulsive_constant | repulsive constant of the artificial potential field algorithm (must be negative) |
| repulsive_threshold | repulsive threshold of the artificial potential field algorithm|
| speed_limit | speed limit |
|error_radius|distance between goal and agent to determine if it has reached goal |
|potential_field_timeout| the time after which the potential field algorithm will give a timeout|

**2-crazyflies.yaml**

Has information about drones used in the crazyflie simulation

| Parameter     |   Explanation |
| ------------- | ------------- |
| channel | the channel from which the drone receives signal|
| id | the id of the drone |
| initialPosition | the initial position of the drone |
| type | drone type |
