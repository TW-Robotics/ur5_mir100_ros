xterm -e "roscore" &
sleep 2
xterm -hold -e "roslaunch mir_gazebo mir_maze_world.launch" &
sleep 2
xterm -hold -e "rosservice call /gazebo/unpause_physics"&
sleep 5
xterm -hold -e "roslaunch mir_gazebo fake_localization.launch delta_x:=-10.0 delta_y:=-10.0"&
sleep 2
xterm -hold -e "roslaunch mir_navigation start_planner.launch \
    map_file:=$(rospack find mir_gazebo)/maps/maze.yaml \
    virtual_walls_map_file:=$(rospack find mir_gazebo)/maps/maze_virtual_walls.yaml \
    local_planner:=eband"&
xterm -hold -e "rviz -d $(rospack find mir_navigation)/rviz/navigation.rviz"&
wait