#!/usr/bin/env python
import time
import roslib
import rospy
import sys
import actionlib
import copy
import tf

import moveit_commander
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from moveit_msgs.msg import RobotTrajectory
from sensor_msgs.msg import JointState
from control_msgs.msg import *
from trajectory_msgs.msg import *
from std_msgs.msg import String
import math
from math import pi

# Sources:
# Moveit-MoveGroup-Commander: http://docs.ros.org/kinetic/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html
# Moveit-Planning-Interface: http://docs.ros.org/kinetic/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroupInterface.html
# Planning Scene: http://docs.ros.org/kinetic/api/moveit_commander/html/classmoveit__commander_1_1planning__scene__interface_1_1PlanningSceneInterface.html
# To make mesh-importer working: https://launchpadlibrarian.net/319496602/patchPyassim.txt
#		open /usr/lib/python2.7/dist-packages/pyassimp/core.py and change line 33 according to the link to "load, release, dll = helper.search_library()"

''' Control the UR5 robot - basic functions (e.g. moveXYZ) and task-specific (e.g. layDown) '''

# Set debug = True to display debug-messages
debug = False

class ur5Controler():
	def __init__(self):
		# Set Robot paramters: position over floor, speed and acceloration [0, 1] (only 0.1 steps)
		self.speed = 0.1
		self.acceleration = 0.1
		self.speedScalingFactor = 0.05		# For timing of path-planning-points [very small eg 0.01, 1]
		self.floor_to_UR = 0 				# Distance = 0 because planning frame is base-footprint

		# Init variables
		self.camToObj = Pose()
		self.baseToObj = Pose()
		self.pickUpHeight = 0 		# Robot stores height over its planning-frame where it picked up object [in m]. e.g. Table = 720mm -> 0.814
		self.distToObj = 1000

		# Set True to make the program ask before the robot moves
		self.checkBeforeDo = True

		# Init moveit_commander
		moveit_commander.roscpp_initialize(sys.argv)
		self.robot = moveit_commander.RobotCommander()
		group_name = "manipulator"
		self.group = moveit_commander.MoveGroupCommander(group_name)
		self.scene = moveit_commander.PlanningSceneInterface()
		self.group.set_end_effector_link("gripper")
		self.group.set_pose_reference_frame("/base_footprint")

		rospy.Subscriber("/tf_objToBase", Pose, self.baseToObj_callback, queue_size=1)	# get transformation from object to base for R1-Move and planning
		rospy.Subscriber("/tf_objToCam", Pose, self.camToObj_callback, queue_size=1)	# get transformation from object to cam for R4-Move

		# Wait for init of subscribers
		rospy.sleep(1)

	def camToObj_callback(self, data):
		self.camToObj = data

	def baseToObj_callback(self, data):
		self.baseToObj = data


	######################################################
	###### GENERAL FUNCTIONS FOR BASIC MOVING TASKS ######
	######################################################

	# Move robot to upright position
	def go_home(self):
		goalPose = [0, -1.565, 0, -1.569, 0, 0]
		self.execute_move(goalPose)

	# Move robot to a specific pose
	def move_to_pose(self, goalPose):
		goal_pose = Pose()

		goal_pose.position.x = goalPose[0]
		goal_pose.position.y = goalPose[1]
		goal_pose.position.z = goalPose[2]
		goal_pose.orientation.x = goalPose[3]
		goal_pose.orientation.y = goalPose[4]
		goal_pose.orientation.y = goalPose[5]
		goal_pose.orientation.w = goalPose[6]

		self.execute_move(goal_pose)

	# Move to x/y/z-position (incremental)
	def move_xyz(self, x_inc, y_inc, z_inc):
		goal_pose = self.group.get_current_pose().pose

		goal_pose.position.x = goal_pose.position.x + x_inc
		goal_pose.position.y = goal_pose.position.y + y_inc
		goal_pose.position.z = goal_pose.position.z + z_inc

		self.execute_move(goal_pose)

	# Move along a cartesian path
	def move_cartesian_path(self, x_inc, y_inc, z_inc):
		# Init waypoints and scale
		waypoints = []
		scale = 1
		wpose = self.group.get_current_pose().pose

		wpose.position.x += scale * x_inc
		wpose.position.y += scale * y_inc
		wpose.position.z += scale * z_inc
		waypoints.append(copy.deepcopy(wpose))

		(plan, fraction) = self.group.compute_cartesian_path(
										   waypoints,   # waypoints to follow
										   0.01,        # eef_step (0.01 means interpolate at a resolution of 1 cm)
										   0.0)         # jump_threshold (0 means disabled)

		# Execute the planned path
		self.execute_plan(plan)

	# Move one specific joint by a specific angle
	def move_joint(self, jointNr, angleDeg_inc):
		# Calculate the angle in radiant
		angleRad_inc = angleDeg_inc * pi / 180

		# Set goal to current joint values and overwrite the relevant angle
		goal_jointStates = self.group.get_current_joint_values()
		goal_jointStates[jointNr] = goal_jointStates[jointNr] + angleRad_inc

		# Call function to move robot
		self.execute_move(goal_jointStates)

	# Move one specific joint to a given goal angle
	def move_joint_to_target(self, jointNr, angleRad):
		# Set goal to current joint values and overwrite the relevant angle
		goal_jointStates = self.group.get_current_joint_values()
		goal_jointStates[jointNr] = angleRad

		# Call function to move robot
		self.execute_move(goal_jointStates)

	# Move the robot to goal pose or orientation
	def execute_move(self, goal):
		self.setSpeed()

		# if the goal is a pose
		if type(goal) is Pose:
			self.group.set_pose_target(goal)
		else:
			self.group.set_joint_value_target(goal)
		plan = self.group.plan()	# Show move in rviz

		rospy.sleep(0.05)	# Give time for keyboard-interrupt
		if self.confirmation(goal):
			self.group.go(wait=True)
			self.group.clear_pose_targets()
			self.group.stop()

	# Move the robot along a specified way (plan)
	def execute_plan(self, plan):
		# Retime all timestamps of the way-points to make the robot move at a specified speed
		plan = self.group.retime_trajectory(self.robot.get_current_state(), plan, self.speedScalingFactor)

		rospy.sleep(0.05)	# Give time for keyboard-interrupt
		if self.confirmation(plan):
			self.group.execute(plan, wait=True)

	# Check, if input necessary, if there is positive input
	def confirmation(self, goal):
		inp = ""
		if self.checkBeforeDo == True:
			if debug == True:
				if type(goal) != RobotTrajectory:
					print " *************************************** Current ***"
					if type(goal) is Pose:
						print self.group.get_current_pose().pose
					else:
						print self.group.get_current_joint_values()
					print " *************************************** Goal ***"
					print goal
			inp = raw_input("Move robot? y/n: ")[0]
		if (inp == 'y' or self.checkBeforeDo == False):
			print "Moving robot..."
			return True
		print "Aborted by user."
		return False

	# Define Speed and acceleration
	def setSpeed(self):
		self.group.set_max_velocity_scaling_factor(self.speed)
		self.group.set_max_acceleration_scaling_factor(self.acceleration)

	# Function to make box at specific position - currently not needed
	def addObject(self):
		rospy.sleep(2)
		obj_pose = PoseStamped()
		obj_pose.header.frame_id = self.robot.get_planning_frame()
		obj_pose.pose.orientation.w = 1.0
		obj_pose.pose.position.x = -0.2
		obj_pose.pose.position.y = 0
		obj_pose.pose.position.z = -0.3
		box_name = "MIR"
		self.scene.add_box(box_name, obj_pose, size=(0.6, 0.4, 0.6))
		rospy.sleep(1)
		
	# Check if a given goal-pose is reachable
	def isReachable(self, goalPose):
		# Change planning-time to make process faster
		oldTime = self.group.get_planning_time()
		self.group.set_planning_time(0.5)
		self.group.set_pose_target(goalPose)
		plan = self.group.plan()
		self.group.clear_pose_targets()
		# Reset planning-time
		self.group.set_planning_time(oldTime)
		# Check if a valid trajectory has been found
		if len(plan.joint_trajectory.joint_names) == 0:
			return False
		return True


	#####################################################
	####### SPECIFIC FUNCTIONS FOR TASK EXECUTION #######
	#####################################################

	# Check if a given goal-pose and pre-goal-pose in zDist is reachable
	def isGoalReachable(self, zDist):
		# Copy current pose
		current_pose = self.group.get_current_pose().pose
		goal_pose = current_pose

		# Check if pre-goal is reachable
		# Calculate orientation of eef when moved over object
		quats = tf.transformations.quaternion_from_euler(pi/2, self.group.get_current_joint_values()[0], -pi/2, 'rxyz')
		goal_pose.orientation.x = quats[0]
		goal_pose.orientation.y = quats[1]
		goal_pose.orientation.z = quats[2]
		goal_pose.orientation.w = quats[3]
		goal_pose.position.x = self.baseToObj.position.x
		goal_pose.position.y = self.baseToObj.position.y
		goal_pose.position.z = self.baseToObj.position.z + float(zDist) / 1000

		if not self.isReachable(goal_pose):
			return False

		# Check if goal is reachable
		goal_pose.position.z = self.baseToObj.position.z

		if not self.isReachable(goal_pose):
			return False
		return True		

	# Center the object by moving joint 1 and 4
	def followObject(self):
		# Joint 1
		act_jointStates = self.group.get_current_joint_values()
		theta = math.atan2(self.baseToObj.position.y, self.baseToObj.position.x)
		beta = math.atan2(self.camToObj.position.z, self.camToObj.position.x)
		a = math.sqrt(self.baseToObj.position.y**2 + self.baseToObj.position.x**2)
		b = math.sqrt(self.camToObj.position.z**2 + self.camToObj.position.x**2)
		delta = math.asin(b/a * math.sin(pi/2 + beta))
		if debug == True:
			print "correction deg: " + str(delta*180/pi)
			print "goal: " + str((act_jointStates[0] - delta)*180/pi)
		#self.move_joint_to_target(0, act_jointStates[0] - delta)

		# Joint 4
		act_jointStates = self.group.get_current_joint_values()
		phi = math.atan2(self.camToObj.position.z, self.camToObj.position.y)
		if debug == True:
			print "correction deg: " + str((pi/2-phi)*180/pi)
			print "goal: " + str((act_jointStates[3] + (pi/2 - phi))*180/pi)
		#self.move_joint_to_target(3, act_jointStates[3] + (pi/2 - phi))

		# Move joints simulatenously
		goal_jointStates = act_jointStates
		goal_jointStates[0] = act_jointStates[0] - delta
		goal_jointStates[3] = act_jointStates[3] + (pi/2 - phi)
		self.execute_move(goal_jointStates)

		# Calculate Distance to object
		self.distToObj = math.sqrt(self.baseToObj.position.x**2 + self.baseToObj.position.y**2 + self.baseToObj.position.z**2)

	# Move the robot to the position, where it starts to search for the object
	def moveToSearchPose(self, orientation, tableHeight):
		# Drive to position where r = 0.4 and h = 0.6
		jointStates = [110*pi/180, -pi/2, pi/2, -110*pi/180, -pi/2, 0]

		if orientation == "left":
			jointStates[0] = 65*pi/180
		elif orientation == "right":
			jointStates[0] = -115*pi/180
		elif orientation == "front":
			jointStates[0] = -25*pi/180

		self.execute_move(jointStates)

		# Correct height dependend on table height
		if tableHeight != 720:
			print "Correcting height..."
			actPose = self.group.get_current_pose().pose
			actPose.position.z = actPose.position.z - float(720) / 1000 + float(tableHeight) / 1000
			self.execute_move(actPose)

	# Turn the robot around R1 and R4 to search the object
	def searchObject(self, num):
		if num == 0:
			self.move_joint(0, 25)
		elif num == 1:
			self.move_joint(0, 25)
		elif num == 2:
			self.move_joint(3, 15)
		elif num == 3:
			self.move_joint(0, -25)
		elif num == 4:
			self.move_joint(0, -25)
		else:
			return False
		return True

	# Move the robot to the pre-goal-pose to analyze the depth-image
	def moveOverObject(self, zDist, tableHeight):
		goal_pose = Pose()

		quats = tf.transformations.quaternion_from_euler(pi/2, self.group.get_current_joint_values()[0], -pi/2, 'rxyz')
		goal_pose.orientation.x = quats[0]
		goal_pose.orientation.y = quats[1]
		goal_pose.orientation.z = quats[2]
		goal_pose.orientation.w = quats[3]
		goal_pose.position.x = self.baseToObj.position.x
		goal_pose.position.y = self.baseToObj.position.y
		#goal_pose.position.z = self.baseToObj.position.z + float(zDist) / 1000
		goal_pose.position.z = float(zDist + tableHeight - self.floor_to_UR) / 1000

		self.execute_move(goal_pose)

	# Move the last joints of the robot to grasp a bottle
	def moveToPreGrabbingPoseBottle(self):
		goal_jointStates = self.group.get_current_joint_values()
		goal_jointStates = [goal_jointStates[0], -70*pi/180, 100*pi/180, -30*pi/180, 85*pi/180, -180*pi/180]
		self.execute_move(goal_jointStates)

	# Move the robot to to the position of the grasping point (first above it) in the correct orientation
	def moveToGrabbingPose(self):
		goal_pose = self.baseToObj
		goal_pose.position.z = goal_pose.position.z + 0.15 # Make EEF stop 15 cm over object
		self.execute_move(goal_pose)
		goal_pose.position.z = goal_pose.position.z - 0.16 # Move down EEF to grasp object
		self.execute_move(goal_pose)
		# Store pickUpHeight to put down object later
		self.pickUpHeight = goal_pose.position.z
		print self.pickUpHeight

	# Move the robot with the object to a safe transport-pose
	def moveToTransportPose(self, objectG):
		if objectG == "bottle":
			jointStates = [0, -160*pi/180, 135*pi/180, 30*pi/180, 90*pi/180, -180*pi/180]
		elif objectG == "cup":
			jointStates = [0, -130*pi/180, 130*pi/180, -90*pi/180, -90*pi/180, 0]

		self.execute_move(jointStates)

	# Move the robot to a safe position to drive around
	def moveToDrivingPose(self):
		jointStates = [0, -180*pi/180, 150*pi/180, -150*pi/180, -90*pi/180, 0]
		self.execute_move(jointStates)

	# Move the robot to lay down the object
	def layDown(self, objectG, side, pickUpTableHeight, putDownTableHeight):
		# Move robot up if put-down height is greater than robots actual pose
		actPose = self.group.get_current_pose().pose
		newPose = self.group.get_current_pose().pose
		newPose.position.z = float(putDownTableHeight + 150) / 1000
		if newPose.position.z > actPose.position.z:
			print "Driving up..."
			self.execute_move(newPose)

		# Turn R1 to put-down place
		moveX = 0
		moveY = 0
		act_jointStates = self.group.get_current_joint_values()
		if side == "left":
			act_jointStates[0] = 90*pi/180
			moveY = 0.5
		elif side == "right":
			act_jointStates[0] = -90*pi/180
			moveY = -0.5
		if side == "front":
			act_jointStates[0] = 0
			moveX = 0.5
		print "Turning to put-down place..."
		self.execute_move(act_jointStates)

		# Drive over the put-down place
		print "Moving over put-down place..."
		self.move_xyz(moveX, moveY, 0)

		# Drive down to putDown-Pose
		actPose = self.group.get_current_pose().pose
		pickUpHeight_overTable = self.pickUpHeight - float(pickUpTableHeight) / 1000
		actPose.position.z = float(putDownTableHeight) / 1000 + pickUpHeight_overTable
		print actPose.position.z
		print "Lying down..."
		self.execute_move(actPose)

	# Calculate distance from TCP to floor in mm
	def tcp_to_floor(self):
		UR_z = self.group.get_current_pose().pose.position.z * 1000 	# mm

		return self.floor_to_UR + UR_z

def main(args):
	try:
		# Initialize ros-node and Class
		rospy.init_node('ur5Controler', anonymous=True, disable_signals=True)
		ur5 = ur5Controler()

		''' # Put Objects into the robots world
		ur5.scene.remove_world_object()
		ur5.attachEEF()
		ur5.addObject()
		
		# Move to up-Position
		print "Moving home"
		ur5.go_home()
		
		# Move to pose
		# Info: Get actual pose: rosrun tf tf_echo base_link tool0
		print "Moving to pose"
		goalPose = [0, 0.191, 0.937, 0.707, 0, 0, 0.707] # Point x, y, z in Meter; Orientation x, y, z, w in Quaternionen
		ur5.move_to_pose(goalPose)
		
		# Move to x/y/z-position (incremental)
		print "Moving xyz-incremental"
		x_inc = 0
		y_inc = 0.1
		z_inc = -0.1
		ur5.move_xyz(x_inc, y_inc, z_inc)

		# Move along a cartesian path
		print "Moving along cartesian path"
		ur5.move_cartesian_path(x_inc, y_inc, z_inc)
		
		# Move to joint-orientations
		print "Moving to joint-orientations"
		jointStates = [-1.13, -1.56, -0.65, -0.94, 1.81, 0.54] # R1-R6
		ur5.execute_move(jointStates)

		# Move one specific joint one specific angle
		print "Moving one joint"
		jointNr = 0			# 0 to 5
		angleDeg_inc = 90
		ur5.move_joint(jointNr, angleDeg_inc)
		
		ur5.go_home()'''
		

	except KeyboardInterrupt:
		rospy.signal_shutdown("KeyboardInterrupt")
		raise

if __name__ == '__main__':
	main(sys.argv)
