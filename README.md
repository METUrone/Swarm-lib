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

- If pycrazyswarm is not found by python run the following:
```
echo >> ~/.bashrc "export PYTHONPATH=$PYTHONPATH:~/crazyswarm/ros_ws/src/crazyswarm/scripts" && . ~/.bashrc
```

