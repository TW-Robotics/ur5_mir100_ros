#!/bin/bash
xterm -hold -e "roscore" &
sleep 3
xterm -hold -e "rosrun coffee_on_table camera_py2ros_wrapper" &
xterm -hold -e "roslaunch coffee_on_table img_rotate.launch" &
xterm -hold -e "roslaunch coffee_on_table yolotest.launch" &
sleep 3
xterm -hold -e "roslaunch coffee_on_table mir_ur5_sim.launch limited:=true" &
sleep 2
xterm -hold -e "roslaunch ur_modern_driver ur5_bringup.launch limited:=true robot_ip:=192.168.12.52" &
sleep 5
xterm -hold -e "roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch limited:=true" &
sleep 2
xterm -hold -e "roslaunch ur5_moveit_config moveit_rviz.launch config:=true" &
sleep 3
xterm -hold -e "roslaunch mir_driver mir.launch" &
sleep 5
xterm -hold -e "roslaunch coffee_on_table tf_transform.launch" &
#--------------------#
#--- We are done! ---#
#--------------------#
echo "Wait for processes ...."
wait