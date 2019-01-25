#!/bin/bash
#clear Terminal

# GENERAL
sudo apt-get update
sudo apt-get upgrade
echo "Please install ROS and openCV before continuing."
#bash installROSOpenCV.sh

# CUDA
echo "Please install Cuda and ROS before continuing."

# PROJECT BUTLER
echo "Downloading Butler-Project"
cd
cd git
git clone https://github.com/JRauer/butler.git
ln -s "$(pwd)/butler" ~/catkin_ws/src/butler		# make link in catkin-folder
cd butler/butler
sudo chmod +x -R src/		# sets all python files to executable

# DARKNET_ROS YOLONET
echo "Downloading YOLOnet"
cd
cd git
git clone --recursive https://github.com/leggedrobotics/darknet_ros.git
ln -s "$(pwd)/darknet_ros" ~/catkin_ws/src/darknet_ros

# INTEL REALSENSE CAMERA
# Info Tutorial: https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md
echo "Installing Realsense Camera"
sudo apt-key adv --keyserver hkp://keys.gnupg.net:80 --recv-key C8B3A55A6F3EFCDE
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u
sudo apt-get install librealsense2*
ln -s "$(pwd)/realsense" ~/catkin_ws/src/realsense
pip install pyrealsense2

# PIP
#sudo apt-get install python-pip

# PYASSIMP - only necessary for model-loading
#sudo pip install pyassimp

# UR5
echo "Downloading UR5-Stuff"
sudo apt install ros-kinetic-moveit
sudo apt-get install ros-kinetic-ur-gazebo
sudo apt-get install ros-kinetic-moveit-commander
sudo apt-get install ros-kinetic-ur5-moveit-config
sudo apt-get install ros-kinetic-ur-msgs

# SOCKET and WEBSOCKET
echo "Installing Websocket for communication with MiR"
pip install websocket
echo "Installing Socket for communication with Gripper"
pip install socket

# NTPDATE
echo "Installing ntp-date for time-syncing"
sudo apt-get install ntpdate

# CATKIN_MAKE
echo "Running catkin_make"
cd
cd catkin_ws
catkin_make
echo "Installation completed!"
