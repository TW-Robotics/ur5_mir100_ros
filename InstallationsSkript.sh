sudo apt-get update
sudo apt-get upgrade
bash installROSOpenCV.sh

// Install Cuda


echo plz install cuda

cd
cd git
git clone --recursive https://github.com/leggedrobotics/darknet_ros.git
ln -s "$(pwd)/darknet_ros" ~/catkin_ws/src/darknet_ros

#Git-Ordner: git clone --recursive https://github.com/leggedrobotics/darknet_ros.git
#Yolonet link in catkin-ordner kopieren
#Git-Ordner: git clone https://github.com/JRauer/butler.git
#link kopieren

git clone https://github.com/JRauer/butler.git
ln -s "$(pwd)/butler" ~/catkin_ws/src/coffee_on_table

#Siehe tut: https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md
sudo apt-key adv --keyserver hkp://keys.gnupg.net:80 --recv-key C8B3A55A6F3EFCDE
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u
sudo apt-get install librealsense2*

#ur5 installation

git clone https://github.com/ThomasTimm/ur_modern_driver
ln -s "$(pwd)/ur_modern_driver" ~/catkin_ws/src/ur_modern_driver

echo damit ur modern driver auf kinetic funktioniert, müssen die Änderungen von hier übernommen werden
echo https://github.com/iron-ox/ur_modern_driver/commit/883070d0b6c0c32b78bb1ca7155b8f3a1ead416c

#Git-Ordner: git clone https://github.com/iron-ox/ur_modern_driver
#link kopieren
#damit ur modern driver auf kinetic funktioniert, müssen diese Änderung übernommen werden
#https://github.com/iron-ox/ur_modern_driver/commit/883070d0b6c0c32b78bb1ca7155b8f3a1ead416c

sudo apt-get install ros-kinetic-ur-gazebo
sudo apt-get install ros-kinetic-moveit-commander
sudo apt-get install ros-kinetic-ur5-moveit-config
sudo apt-get install ros-kinetic-ur-msgs

cd butler/coffee_on_table
sudo chmod +x -R src/

#sets all python files to executable

#chmod +x find_mug_on_table/findover.py
#chmod +x tf_transform/camera_tf_broadcaster.py
#chmod +x tf_transform/camera_tf_listener.py
#


