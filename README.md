# Butler
Repository for the Butler project of the UAS Technikum-Wien.

Run InstallationsSkript.sh to install all necessary packages.

For further explanations see Butler_Bedienungsanleitung_v1-1.pdf

### Necessary software ###
- **butler** (including mir_robot, universal_robot, ur_modern_driver)
- **websocket** (python) for communication with MiR
- **moveit** to control UR5
- **socket** (python) for communication with Gripper
- **librealsense2** and **pyrealsense2** (python) for communication with Camera
- **darknet_ros** for object-detection
- **ntpdate** to sync the times between ROS-Cores

### Change and add goals ###
- Open `~/catkin_ws/src/butler/src/mission_control/mission_control.py`
- In function createGoal add a new elif with your goal parameters
  - Set a name
  - Get the MiR-Goal from the web-interface by clicking a position in the map
  - Measure the table or platform height
  - Add the orientation the UR should turn to search / put-down the object

### Run Butler-Robot ###
1. Check connections as mentioned in Butler_Bedienungsanleitung_v1-1.pdf
2. Synchronise times
`bash ~/catkin_ws/src/butler/syncTime.sh`
3. Start basic nodes
`bash ~/catkin_ws/src/butler/launch/run_real_butler.bash`
4. Start task-execution
`rosrun butler robot_control pickUpGoalName putDownGoalName objectToSearch`
  - Use goalNames given in mission_control.py
  - objectToSearch for must be cup or bottle