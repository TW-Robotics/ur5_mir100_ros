#!/bin/bash
xterm -hold -e "roscore" &
sleep 3
#xterm -hold -e "roslaunch realsense2_camera demo_pointcloud.launch" &
xterm -hold -e "rosrun butler camera_py2ros_wrapper" &
xterm -hold -e "roslaunch butler img_rotate.launch" &
xterm -hold -e "roslaunch butler yolonet.launch" &
sleep 3
xterm -hold -e "roslaunch butler butler_robot.launch" &
sleep 2
xterm -hold -e "roslaunch ur_modern_driver ur5_bringup.launch limited:=true robot_ip:=192.168.12.52" &
sleep 2
xterm -hold -e "roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch" &
sleep 2
xterm -hold -e "roslaunch ur5_moveit_config moveit_rviz.launch config:=true" &
sleep 3
xterm -hold -e "roslaunch mir_driver mir.launch" &
sleep 15
xterm -hold -e "roslaunch butler tf_transform.launch" &
#--------------------#
#--- We are done! ---#
#--------------------#
echo "Wait for processes ...."
wait