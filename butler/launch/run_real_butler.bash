#!/bin/bash

# Skript to launch the complete butler-robot
echo "Starting roscore..."
xterm -hold -e "roscore" &
sleep 3

echo "Launching basic nodes..."
echo "Launching camera and neural network..."
xterm -hold -e "rosrun butler camera_py2ros_wrapper" &
xterm -hold -e "roslaunch butler img_rotate.launch" &
xterm -hold -e "roslaunch butler yolonet.launch" &
sleep 3

echo "Uploading robot-model to core..."
xterm -hold -e "roslaunch butler butler_robot.launch" &
sleep 2

echo "Connecting to UR5 and launching path-planner..."
xterm -hold -e "roslaunch ur_modern_driver ur5_bringup.launch limited:=true robot_ip:=192.168.12.52" &
sleep 2
xterm -hold -e "roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch" &
sleep 2
xterm -hold -e "roslaunch ur5_moveit_config moveit_rviz.launch config:=true" &
sleep 3

echo "Connecting to MiR100..."
xterm -hold -e "roslaunch mir_driver mir.launch" &
sleep 15

echo "Launching nodes for tf-transformation..."
xterm -hold -e "roslaunch butler tf_transform.launch" &

echo "We are done. You can now rosrun butler robot_control."
wait