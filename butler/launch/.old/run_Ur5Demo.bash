#!/bin/bash
xterm -hold -e "roscore" &
sleep 3
xterm -hold -e "roslaunch ur_gazebo ur5_only.launch limited:=true" &
sleep 5
xterm -hold -e "roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch limited:=true sim:=true" &
sleep 2
xterm -hold -e "roslaunch ur5_moveit_config moveit_rviz.launch config:=true" &
#xterm -hold -e "roslaunch butler tf_transform.launch" &
#xterm -hold -e "rosrun butler ur5_control" &
#--------------------#
#--- We are done! ---#
#--------------------#
echo "Wait for processes ...."
wait
