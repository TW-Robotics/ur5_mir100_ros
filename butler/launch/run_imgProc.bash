#!/bin/bash
echo "Running ROS stuff..."
#echo "Starting roscore"
xterm -hold -e "roscore" &
sleep 3
#xterm -hold -e "roslaunch src/yolonet/usbcam.launch" &
#xterm -hold -e "roslaunch butler bringUp_camera.launch" &
xterm -hold -e "rosrun butler camera_py2ros_wrapper" &
xterm -hold -e "roslaunch butler img_rotate.launch" &
xterm -hold -e "roslaunch butler yolonet.launch" &
#xterm -hold -e "roslaunch butler tf_transform.launch" &
xterm -geometry 96x24+0-0 -hold -e "rosrun butler img_processing cup " &	# Arguments: InnerClass, OuterClass, Strictness (outside, touching (means center of inner box is in outer box), inside)
#xterm -geometry 96x24+0-0 -hold -e "rosrun butler robot_control" &
#--------------------#
#--- We are done! ---#
#--------------------#
echo "Wait for processes ...."
wait
