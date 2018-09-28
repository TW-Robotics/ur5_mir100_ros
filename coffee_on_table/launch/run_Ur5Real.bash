#!/bin/bash
# ROUTER xterm -hold -e "roslaunch ur_modern_driver ur5_bringup.launch limited:=true robot_ip:=192.168.4.103" &
xterm -hold -e "roslaunch ur_modern_driver ur5_bringup.launch limited:=true robot_ip:=192.168.12.52" &
sleep 5
xterm -hold -e "roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch limited:=true" &
sleep 2
xterm -hold -e "roslaunch ur5_moveit_config moveit_rviz.launch config:=true" &
#xterm -hold -e "rosrun coffee_on_table ur5_control" &
#--------------------#
#--- We are done! ---#
#--------------------#
echo "Wait for processes ...."
wait
