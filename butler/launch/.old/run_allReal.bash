#!/bin/bash
echo "Running ROS stuff..."
#echo "Starting roscore"
#xterm -e "roscore" &
#sleep 2
#xterm -hold -e "roslaunch butler bringUp_camera.launch" &
xterm -hold -e "rosrun butler camera_py2ros_wrapper" &
xterm -hold -e "roslaunch butler img_rotate.launch" &
xterm -hold -e "roslaunch butler yolonet.launch" &
#xterm -geometry 96x24+0-0 -hold -e "rosrun butler img_processing cup" &

xterm -hold -e "roslaunch ur_modern_driver ur5_bringup.launch limited:=true robot_ip:=192.168.12.52" &
sleep 5
xterm -hold -e "roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch limited:=true" &
sleep 2
xterm -hold -e "roslaunch ur5_moveit_config moveit_rviz.launch config:=true" &
xterm -hold -e "roslaunch butler tf_transform.launch" &
#--------------------#
#--- We are done! ---#
#--------------------#
echo "Wait for processes ...."
wait
