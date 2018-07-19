#!/bin/bash
echo "Running ROS stuff..."
echo "Starting roscore"
xterm -e "roscore" &
sleep 2
#xterm -hold -e "roslaunch src/yolotest/usbcam.launch" &
xterm -hold -e "roslaunch realsense2_camera get_depth_cam.launch" &
#xterm -hold -e "roslaunch realsense2_camera rs_aligned_depth.launch" &
xterm -hold -e "roslaunch coffee_on_table yolotest.launch" &
xterm -geometry 96x24+0-0 -hold -e "rosrun coffee_on_table cupfind person cup inside" &	# Arguments: OuterClass, InnerClass, Strictness (outside, touching (means center of inner box is in outer box), inside)
#--------------------#
#--- We are done! ---#
#--------------------#
echo "Wait for processes ...."
wait
