#!/bin/bash

# Skript to launch the camera-node and yolonet
echo "Starting roscore"
xterm -hold -e "roscore" &
sleep 3
xterm -hold -e "rosrun butler camera_py2ros_wrapper" &
xterm -hold -e "roslaunch butler img_rotate.launch" &
xterm -hold -e "roslaunch butler yolonet.launch" &
#--------------------#
#--- We are done! ---#
#--------------------#
echo "Wait for processes ...."
wait
