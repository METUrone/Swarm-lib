# Gazebo Simulation for Swarm-lib

**Install Dependecies**

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
```
```
echo "source PX4-Autopilot/Tools/setup_gazebo.bash $(pwd)/PX4-Autopilot $(pwd)/PX4-Autopilot/build/px4_sitl_default" >> ~/.bashrc
echo "export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/PX4-Autopilot" >> .bashrc
echo "export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/PX4-Autopilot/Tools/sitl_gazebo" >> ~/.bashrc

source ~/.bashrc
```
**Install Swarm-lib (Our Package)**

```
mkdir -p Swarm_ws/src && cd Swarm_ws/src
git clone --branch gazebo https://github.com/METUrone/Swarm-lib.git --single-branch
cd ~/Swarm_ws
catkin_make
```

**Launch the Simulation**
```console
catkin_make
source devel/setup.bash
roslaunch swarm gazebo.launch
```
