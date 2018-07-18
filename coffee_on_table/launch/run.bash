#!/bin/bash
echo "Running ROS stuff..."
echo "Starting roscore"
xterm -e "roscore" &
sleep 2
#xterm -hold -e "roslaunch src/yolotest/usbcam.launch" &
xterm -hold -e "roslaunch realsense2_camera rs_camera.launch" &
xterm -hold -e "roslaunch src/yolotest/yolotest.launch" &
xterm -hold -e "rosrun coffee_on_table cupfind" &
#--------------------#
#--- We are done! ---#
#--------------------#
echo "Wait for processes ...."
wait
