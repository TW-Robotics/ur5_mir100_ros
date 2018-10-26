#!/bin/bash
xterm -hold -e "roscore" &
sleep 3
xterm -hold -e "roslaunch coffee_on_table mir_ur5_sim.launch limited:=true" &
sleep 2

# Further UR-Stuff
xterm -hold -e "roslaunch ur_gazebo ur5.launch limited:=true" &
sleep 5
xterm -hold -e "roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch limited:=true sim:=true" &
sleep 2
xterm -hold -e "roslaunch ur5_moveit_config moveit_rviz.launch config:=true" &


# Further MiR-Stuff
#xterm -hold -e "roslaunch mir_gazebo mir_maze_world.launch" &
#sleep 5
#xterm -hold -e "rosservice call /gazebo/unpause_physics"&
#sleep 5
#xterm -hold -e "roslaunch mir_gazebo fake_localization.launch delta_x:=-10.0 delta_y:=-10.0"&
#sleep 5
#xterm -hold -e "roslaunch mir_navigation start_planner.launch \
#    map_file:=$(rospack find mir_gazebo)/maps/maze.yaml \
#    virtual_walls_map_file:=$(rospack find mir_gazebo)/maps/maze_virtual_walls.yaml \
#    local_planner:=eband"&
#xterm -hold -e "rviz -d $(rospack find mir_navigation)/rviz/navigation.rviz"&


#--------------------#
#--- We are done! ---#
#--------------------#
echo "Wait for processes ...."
wait

