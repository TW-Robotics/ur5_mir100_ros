#!/bin/bash
#clear Terminal

printf "\033c"
usr=$(whoami)

echo "                                                                                
       //////////////////////////
      ///////.     // *///* *////
     *///////. ////// *///* *////
     ////////.    *//       *////
    ,////////. ////// *///* *////
    *////////. ////// *///* *////
    ,////////////////////////////
                                                                                                   
     (((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((/    
     .(((((((       (/     (((     /(  ((((  ((   (((. ((* ((( *((  ((  ((((  ((/  ((((  *((((/    
      /((((((((( ,(((/ (((((  (((((((  ((((  (( .  ((. ((* ((( ** *(((  ((((  ((.  .((.  .((((/    
       ((((((((( ,(((/    (( ,(((((((        ((  (  (. ((* (((   (((((  ((((  (( ,* (/ /  ((((/    
        (((((((( ,(((/ (((((  (((((((  ((((  ((  ((    ((* ((( *(  (((  ((((  (( /(   .(/ /(((/    
         ((((((( ,(((/     (((     /(  ((((  ((  (((,  ((* ((( *((  /((,    ,((, (((  ((( .(((/    
          ((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((/    
                                                                                                   
          .%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%.                                              
           %%%/ #%%(  %%%  %( #%#     %%  /%%% ,%%%%.                                              
           %%%%  %% , (%( #%( #%# (%%%%% * ,%% ,#%%%.                                              
          *%%%%/ #* %  %  #%( #%#    %%% ((  % ,#%%%.                                              
          #%%%%% . /%# * #%%( #%# (%%%%% (%#   ,#%%%.                                              
         .%%%%%%(  %%%. .%%%( #%#     %% (%%%  ,#%%%.                                              
         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%.
              "

# GENERAL
sudo apt-get update
sudo apt-get upgrade
bash installROSOpenCV.sh

# CUDA
echo "Please install Cuda and ROS before continuing"

# PROJECT BUTLER
echo "Downloading Butler-Project"
cd
cd git
git clone https://github.com/JRauer/butler.git
ln -s "$(pwd)/butler" ~/catkin_ws/src/coffee_on_table		# make link in catkin-folder
cd butler/coffee_on_table
sudo chmod +x -R src/		#sets all python files to executable

# DARKNET_ROS YOLONET
echo "Downloading YOLOnet"
cd
cd git
git clone --recursive https://github.com/leggedrobotics/darknet_ros.git
ln -s "$(pwd)/darknet_ros" ~/catkin_ws/src/darknet_ros

# INTEL REALSENSE CAMERA
# Info Tutorial: https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md
echo "Downloading Realsense"
sudo apt-key adv --keyserver hkp://keys.gnupg.net:80 --recv-key C8B3A55A6F3EFCDE
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u
sudo apt-get install librealsense2*
ln -s "$(pwd)/realsense" ~/catkin_ws/src/realsense

# PIP
#sudo apt-get install python-pip

# PYASSIMP
#sudo pip install pyassimp

# UR5
echo "Downloading UR5-Stuff"
git clone https://github.com/ThomasTimm/ur_modern_driver
ln -s "$(pwd)/ur_modern_driver" ~/catkin_ws/src/ur_modern_driver
echo "ATTENTION: To make ur-driver on ros kinetic work, make the following changes:"
echo "https://github.com/iron-ox/ur_modern_driver/commit/883070d0b6c0c32b78bb1ca7155b8f3a1ead416c"

echo "ATTENTION: To make model-loading with pyassimp on ros kinetic work, make the following change:"
echo "https://launchpadlibrarian.net/263969718/patch.txt"
echo "In File '/usr/lib/python2.7/dist-packages/pyassimp/core.py' change line 33 to 'load, release, dll = helper.search_library()'"

sudo apt-get install ros-kinetic-ur-gazebo
sudo apt-get install ros-kinetic-moveit-commander
sudo apt-get install ros-kinetic-ur5-moveit-config
sudo apt-get install ros-kinetic-ur-msgs

#To give the robot the gripper-eef (only for planning) copy the files from UR5-Files:
#	FILE 					COPY TO
#	ur5.urdf.xacro			  /opt/ros/kinetic/share/ur_description/urdf
#	ur5.srdf				      /opt/ros/kinetic/share/ur5_moveit_config/config
#	moveit_rviz.launch 		/opt/ros/kinetic/share/ur5_moveit_config/launch
#	moveitTF.rviz 			  /opt/ros/kinetic/share/ur5_moveit_config/launch
# move_group.launch     /opt/ros/kinetic/share/ur5_moveit_config/launch
# ur5.launch            /opt/ros/kinetic/share/ur_gazebo/launch
#   ur5_joint_limited_robot.urdf.xacro    /opt/ros/kinetic/share/ur_description/urdf
#   ur5_robot.urdf.xacro    /opt/ros/kinetic/share/ur_description/urdf
sudo cp /home/mluser/git/butler/coffee_on_table/ur5_Files/ur5.urdf.xacro /opt/ros/kinetic/share/ur_description/urdf
sudo cp /home/mluser/git/butler/coffee_on_table/ur5_Files/ur5.srdf /opt/ros/kinetic/share/ur5_moveit_config/config
sudo cp /home/mluser/git/butler/coffee_on_table/ur5_Files/moveit_rviz.launch /opt/ros/kinetic/share/ur5_moveit_config/launch
sudo cp /home/mluser/git/butler/coffee_on_table/ur5_Files/moveitTF.rviz /opt/ros/kinetic/share/ur5_moveit_config/launch
sudo cp /home/mluser/git/butler/coffee_on_table/ur5_Files/move_group.launch /opt/ros/kinetic/share/ur5_moveit_config/launch
sudo cp /home/mluser/git/butler/coffee_on_table/ur5_Files/ur5.launch /opt/ros/kinetic/share/ur_gazebo/launch
sudo cp /home/mluser/git/butler/coffee_on_table/ur5_Files/ur5_joint_limited_robot.urdf.xacro /opt/ros/kinetic/share/ur_description/urdf
sudo cp /home/mluser/git/butler/coffee_on_table/ur5_Files/ur5_robot.urdf.xacro /opt/ros/kinetic/share/ur_description/urdf

# MOVEIT
sudo apt install ros-kinetic-moveit

catkin_make
echo "Installation completed!"
