#!/bin/bash
xterm -hold -e "roslaunch ur_gazebo ur5.launch limited:=true" &
sleep 5
xterm -hold -e "roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch limited:=true sim:=true" &
sleep 2
xterm -hold -e "roslaunch ur5_moveit_config moveit_rviz.launch config:=true" &
xterm -hold -e "roslaunch coffee_on_table tf_transform.launch" &
#xterm -hold -e "rosrun coffee_on_table ur5_control" &
#--------------------#
#--- We are done! ---#
#--------------------#
echo "Wait for processes ...."
wait
