#!/usr/bin/env python
import time
import math
import roslib; #roslib.load_manifest('ur_driver')
import rospy
import sys
import actionlib
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from moveit_msgs.msg import RobotState
from moveit_msgs.msg import RobotTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from control_msgs.msg import *
from trajectory_msgs.msg import *
from math import pi


# Source: http://docs.ros.org/kinetic/api/moveit_commander/html/move__group_8py_source.html
# Class behind: http://docs.ros.org/kinetic/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroupInterface.html
# Planning Scene: http://docs.ros.org/kinetic/api/moveit_commander/html/classmoveit__commander_1_1planning__scene__interface_1_1PlanningSceneInterface.html#a8c646437964759c78591394867b6c2b9
# In order to make the mesh-importer working: https://launchpadlibrarian.net/319496602/patchPyassim.txt
#		open /usr/lib/python2.7/dist-packages/pyassimp/core.py and change line 33 according to the link to "load, release, dll = helper.search_library()"

# Set True to make the program ask before the robot moves
checkBeforeDo = True

# Class to control and move the ur5-robot
class ur5Controler(object):

	distToObj = 1000

	def __init__(self):
		super(ur5Controler, self).__init__()

		# Set Robot speed and acceloration [0, 1] (only 0.1 steps)
		self.speed = 0.1
		self.acceleration = 0.1
		self.speedScalingFactor = 0.05		# for timing of path-planning-points [very small eg 0.01, 1]

		# Init moveit_commander
		moveit_commander.roscpp_initialize(sys.argv)
		self.robot = moveit_commander.RobotCommander()
		group_name = "manipulator"
		self.group = moveit_commander.MoveGroupCommander(group_name)
		self.scene = moveit_commander.PlanningSceneInterface()

		# Publisher for Robot-Trajectory
		self.trajectory_pub = rospy.Publisher('/move_group/planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
		rospy.Subscriber("/tf_objToBase", Pose, self.baseToObj_callback, queue_size=1)	# get transformation from object to base for R1-Move
		rospy.Subscriber("/tf_objToCam", Pose, self.camToObj_callback, queue_size=1)	# get transformation from object to cam for R4-Move
		self.camToObj = Pose()
		self.baseToObj = Pose()

	def camToObj_callback(self, data):
		self.camToObj = data

	def baseToObj_callback(self, data):
		self.baseToObj = data

	def addObject(self):
		obj_pose = geometry_msgs.msg.PoseStamped()
		obj_pose.header.frame_id = self.robot.get_planning_frame()
		obj_pose.pose.orientation.w = 1.0
		obj_pose.pose.position.x = -0.2
		obj_pose.pose.position.y = 0
		obj_pose.pose.position.z = -0.3
		box_name = "MIR"
		self.scene.add_box(box_name, obj_pose, size=(0.6, 0.4, 0.6))

	def attachEEF(self):
		rospy.sleep(2)
		# Define the pose at the end of the robot
		eef_pose = geometry_msgs.msg.PoseStamped()
		eef_pose.header.frame_id = self.robot.get_planning_frame()
		eef_pose.pose = self.group.get_current_pose().pose
		#eef_pose.pose.position.y = eef_pose.pose.position.y - 0.144

		# Import the STL-Files
		# TODO Make path to parameter
		# x comes out of EEF, Y shows upwords, Z to the left (front view)		
		self.scene.add_mesh("gripper", eef_pose, "/mnt/data/mluser/catkin_ws/src/coffee_on_table/stl_Files/Greifer_mit_Flansch.STL",size=(0.001, 0.001, 0.001))
		self.scene.add_mesh("cam", eef_pose, "/mnt/data/mluser/catkin_ws/src/coffee_on_table/stl_Files/Camera_mit_Halterung.STL",size=(0.001, 0.001, 0.001))
		rospy.sleep(1)
		#print self.scene.get_known_object_names()

		# Attach the EEF to the robot
		eef_link = self.group.get_end_effector_link()
		self.scene.attach_mesh(eef_link, "gripper")
		self.scene.attach_mesh(eef_link, "cam")

	def moveToSearchPose(self):
		# drive to position where r = 0.4 and h = 0.6
		#jointStates = [-0.0258, -0.098, -1.781, -1.262, -1.671, 1.57] # R1-R6 R1: -0.258
		
		jointStates = [0, -pi/2, pi/2, -2.79, -pi/2, pi/2]
		jointStates = [50*pi/180, -pi/2, pi/2, -120*pi/180, -pi/2, pi/2]
		#jointStates = [-0.0258, -0.09668, 0.17604, -1.262, -1.21092, 1.57]
		#jointStates = [-pi+0.01, -0.098, -1.781, -1.262, -1.671, 1.57] # R1-R6
		self.execute_move(jointStates)

	def followObject(self):
		#self.moveToSearchPose()
		#rospy.sleep(5)

		#while True:
			print "move joint 1"
			act_jointStates = self.group.get_current_joint_values()
			theta = math.atan2(self.baseToObj.position.y, self.baseToObj.position.x)
			beta = math.atan2(self.camToObj.position.z, self.camToObj.position.x)
			a = math.sqrt(self.baseToObj.position.y**2 + self.baseToObj.position.x**2)
			b = math.sqrt(self.camToObj.position.z**2 + self.camToObj.position.x**2)
			delta = math.asin(b/a * math.sin(pi/2 + beta))
			print "correction deg: " + str(delta*180/pi)
			print "goal: " + str((act_jointStates[0] - delta)*180/pi)
			#self.move_joint_to_target(0, act_jointStates[0] - delta)

			print "move joint 4"
			act_jointStates = self.group.get_current_joint_values()
			phi = math.atan2(self.camToObj.position.z, self.camToObj.position.y)
			print "correction deg: " + str((pi/2-phi)*180/pi)
			print "goal: " + str((act_jointStates[3] + (pi/2 - phi))*180/pi)
			#self.move_joint_to_target(3, act_jointStates[3] + (pi/2 - phi))

			# Move joints simulatenously
			goal_jointStates = act_jointStates
			goal_jointStates[0] = act_jointStates[0] - delta
			goal_jointStates[3] = act_jointStates[3] + (pi/2 - phi)
			self.execute_move(goal_jointStates)

			self.distToObj = math.sqrt(self.baseToObj.position.x**2 + self.baseToObj.position.y**2 + self.baseToObj.position.z**2)
			print "Distance to obj: " + str(self.distToObj)

			#if self.distToObj <= 0.8:
				#print "Distance to obj: " + str(self.distToObj)
			#	self.moveOverObject()

			'''print "move joint 5"
			act_jointStates = self.group.get_current_joint_values()
			gamma = math.atan2(self.camToObj.position.z, self.camToObj.position.x)
			print "correction deg: " + str((pi/2-gamma)*180/pi)
			print "goal: " + str((act_jointStates[4] - (pi/2 - gamma))*180/pi)
			self.move_joint_to_target(4, act_jointStates[4] - (pi/2 - gamma))		
			'''

	def moveOverObject(self, zDist):
		goal_pose = self.baseToObj 		# Stores pose of object relative to robot frame
		current_pose = self.group.get_current_pose().pose
		goal_pose = current_pose
		#goal_pose.orientation.x = 0.5
		#goal_pose.orientation.y = 0.5
		#goal_pose.orientation.z = -0.5
		#goal_pose.orientation.w = 0.5

		quats = tf.transformations.quaternion_from_euler(pi/2, self.group.get_current_joint_values()[0], -pi/2, 'rxyz')
		goal_pose.orientation.x = quats[0]
		goal_pose.orientation.y = quats[1]
		goal_pose.orientation.z = quats[2]
		goal_pose.orientation.w = quats[3]
		goal_pose.position.x = self.baseToObj.position.x
		goal_pose.position.y = self.baseToObj.position.y
		goal_pose.position.z = self.baseToObj.position.z + float(zDist) / 1000

		self.execute_move(goal_pose)
		#self.move_joint_to_target(5, pi/2)

	def moveToGrabbingPose(self, alpha):
		#goal_pose = self.baseToObj
		#goal_pose.position.x = self.group.get_current_pose().pose.position.x
		#goal_pose.position.y = self.group.get_current_pose().pose.position.y
		#goal_pose.position.z = self.group.get_current_pose().pose.position.z
		#self.execute_move(goal_pose)
		
		#actStates = self.group.get_current_joint_values()
		#print actStates[5]
		#print alpha
		#print "move to " + str(alpha - actStates[5])
		#self.move_joint_to_target(5, alpha + actStates[5])
		goal_pose = self.baseToObj
		self.execute_move(goal_pose)

	def searchObject(self):
		#while self.camToObj.position.x == 0 and self.camToObj.position.y == 0 and self.camToObj.position.z == 0:
			#print self.camToObj
			self.move_joint(0, 25)

	def correctPositionXY(self, x_goal, y_goal):
		goal_pose = self.group.get_current_pose().pose
		goal_pose.position.x = x_goal/1000
		goal_pose.position.y = y_goal/1000
		self.execute_move(goal_pose)

	# Publish the robot's trajectory
	def display_trajectory(self, plan):
		#rospy.sleep(2)
		traj = moveit_msgs.msg.DisplayTrajectory()
		traj.trajectory_start = self.robot.get_current_state()
		traj.trajectory.append(plan)
		self.trajectory_pub.publish(traj)

	# Move robot to upright position
	def go_home(self):
		# Upright position: 0.005937059875577688, -1.5655563513385218, -0.00637227693666631, -1.5696209112750452, 0.009078050963580608, 0.01515068206936121]
		goalPose = [0, -1.565, 0, -1.569, 0, 0]
		self.execute_move(goalPose)

	# Move robot to a specific pose
	def move_to_pose(self, goalPose):
		goal_pose = geometry_msgs.msg.Pose()

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

	# Move one specific joint one specific angle
	def move_joint(self, jointNr, angleDeg_inc):
		# Calculate the angle in radiant
		angleRad_inc = angleDeg_inc * pi / 180

		# Set goal to current joint values and overwrite the relevant angle
		goal_jointStates = self.group.get_current_joint_values()
		goal_jointStates[jointNr] = goal_jointStates[jointNr] + angleRad_inc

		# Call function to move robot
		self.execute_move(goal_jointStates)

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
		self.display_trajectory(plan)

		rospy.sleep(0.05)	# Give time for keyboard-interrupt
		if self.confirmation(goal):
			self.group.go(wait=True)
			self.group.clear_pose_targets()
			self.group.stop()

	# Move the robot along a specified way (plan)
	def execute_plan(self, plan):	#TODO
		# Retime all timestamps of the way-points to make the robot move at a specified speed
		plan = self.group.retime_trajectory(self.robot.get_current_state(), plan, self.speedScalingFactor)
		self.display_trajectory(plan)

		rospy.sleep(0.05)	# Give time for keyboard-interrupt
		if self.confirmation(plan):
			self.group.execute(plan, wait=True)

	# Check, if input necessary, if there is positive input
	def confirmation(self, goal):
		inp = ""
		if (checkBeforeDo):
			if type(goal) != RobotTrajectory:
				print " *************************************** Current ***"
				if type(goal) is Pose:
					print self.group.get_current_pose().pose
				else:
					print self.group.get_current_joint_values()
				print " *************************************** Goal ***"
				print goal
			inp = raw_input("Move robot? y/n: ")[0]
		if (inp == 'y' or checkBeforeDo == False):
			print "Moving robot..."
			return True
		print "Aborted"
		return False

	# Define Speed and acceleration
	def setSpeed(self):
		self.group.set_max_velocity_scaling_factor(self.speed)
		self.group.set_max_acceleration_scaling_factor(self.acceleration)

def main(args):
	try:
		# Initialize ros-node and Class
		rospy.init_node('ur5-controler', anonymous=True)
		ur5 = ur5Controler()

		ur5.scene.remove_world_object()
		ur5.attachEEF()
		ur5.addObject()

		ur5.moveToSearchPose()
		ur5.searchObject()
		ur5.followObject()

		#rospy.spin()

		#while True:
		#	ur5.execute_move(desiredPose)
		#	rospy.sleep(2)

		
		# Move to up-Position
		'''print "Moving home"
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
		'''
		#ur5.go_home()
		

	except KeyboardInterrupt:
		rospy.signal_shutdown("KeyboardInterrupt")
		raise

if __name__ == '__main__':
	main(sys.argv)
