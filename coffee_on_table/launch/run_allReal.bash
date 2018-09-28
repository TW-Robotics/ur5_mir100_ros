#!/bin/bash
echo "Running ROS stuff..."
#echo "Starting roscore"
#xterm -e "roscore" &
#sleep 2
#xterm -hold -e "roslaunch coffee_on_table bringUp_camera.launch" &
xterm -hold -e "rosrun coffee_on_table camera_py2ros_wrapper" &
xterm -hold -e "roslaunch coffee_on_table img_rotate.launch" &
xterm -hold -e "roslaunch coffee_on_table yolotest.launch" &
#xterm -geometry 96x24+0-0 -hold -e "rosrun coffee_on_table cupfind cup" &

xterm -hold -e "roslaunch ur_modern_driver ur5_bringup.launch limited:=true robot_ip:=192.168.4.103" &
sleep 5
xterm -hold -e "roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch limited:=true" &
sleep 2
xterm -hold -e "roslaunch ur5_moveit_config moveit_rviz.launch config:=true" &
xterm -hold -e "roslaunch coffee_on_table tf_transform.launch" &
#--------------------#
#--- We are done! ---#
#--------------------#
echo "Wait for processes ...."
wait
