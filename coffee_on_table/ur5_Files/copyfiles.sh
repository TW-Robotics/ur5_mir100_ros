#!/bin/bash
#clear Terminal

printf "\033c"
usr=$(whoami)

sudo cp ~/git/butler/coffee_on_table/ur5_Files/ur5.urdf.xacro /opt/ros/kinetic/share/ur_description/urdf
sudo cp ~/git/butler/coffee_on_table/ur5_Files/ur5.srdf /opt/ros/kinetic/share/ur5_moveit_config/config
sudo cp ~/git/butler/coffee_on_table/ur5_Files/moveit_rviz.launch /opt/ros/kinetic/share/ur5_moveit_config/launch
sudo cp ~/git/butler/coffee_on_table/ur5_Files/moveitTF.rviz /opt/ros/kinetic/share/ur5_moveit_config/launch
sudo cp ~/git/butler/coffee_on_table/ur5_Files/move_group.launch /opt/ros/kinetic/share/ur5_moveit_config/launch
sudo cp ~/git/butler/coffee_on_table/ur5_Files/ur5.launch /opt/ros/kinetic/share/ur_gazebo/launch
sudo cp ~/git/butler/coffee_on_table/ur5_Files/ur5_joint_limited_robot.urdf.xacro /opt/ros/kinetic/share/ur_description/urdf
sudo cp ~/git/butler/coffee_on_table/ur5_Files/ur5_robot.urdf.xacro /opt/ros/kinetic/share/ur_description/urdf