#!/bin/bash
echo "Running ROS stuff..."
#echo "Starting roscore"
xterm -hold -e "roscore" &
sleep 3
#xterm -hold -e "roslaunch src/yolotest/usbcam.launch" &
#xterm -hold -e "roslaunch coffee_on_table bringUp_camera.launch" &
xterm -hold -e "rosrun coffee_on_table camera_py2ros_wrapper" &
xterm -hold -e "roslaunch coffee_on_table img_rotate.launch" &
xterm -hold -e "roslaunch coffee_on_table yolotest.launch" &
#xterm -hold -e "roslaunch coffee_on_table tf_transform.launch" &
xterm -geometry 96x24+0-0 -hold -e "rosrun coffee_on_table cupfind cup " &	# Arguments: InnerClass, OuterClass, Strictness (outside, touching (means center of inner box is in outer box), inside)
#xterm -geometry 96x24+0-0 -hold -e "rosrun coffee_on_table robot_control" &
#--------------------#
#--- We are done! ---#
#--------------------#
echo "Wait for processes ...."
wait
