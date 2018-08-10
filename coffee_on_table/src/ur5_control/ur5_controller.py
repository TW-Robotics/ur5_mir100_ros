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

# Set True to make the program ask before the robot moves
checkBeforeDo = True

onceFlag = False

# Class to control and move the ur5-robot
class ur5Controler(object):
	wrist1ToObj = Pose()
	def __init__(self):
		super(ur5Controler, self).__init__()

		# Set Robot speed and acceloration [0, 1] (only 0.1 steps)
		self.speed = 1
		self.acceleration = 1
		self.speedScalingFactor = 0.05		# for timing of path-planning-points [very small eg 0.01, 1]

		# Init moveit_commander
		moveit_commander.roscpp_initialize(sys.argv)
		self.robot = moveit_commander.RobotCommander()
		group_name = "manipulator"
		self.group = moveit_commander.MoveGroupCommander(group_name)

		# Publisher for Robot-Trajectory
		self.trajectory_pub = rospy.Publisher('/move_group/planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
		rospy.Subscriber("/tf_objToBase", Pose, self.baseToObj_callback, queue_size=1)
		rospy.Subscriber("/tf_objToWrist1Pub", Pose, self.wrist1ToObj_callback, queue_size=1)

	def wrist1ToObj_callback(self, data):
		self.wrist1ToObj = data

	def baseToObj_callback(self, data):
		global desiredPose
		global onceFlag
		desiredPose = Pose()
		theta = math.atan2(data.position.y, data.position.x)

		r = 0.4
		h = 0.6

		desiredPose.position.x = r*math.sin(theta)
		desiredPose.position.y = r*math.cos(theta)
		desiredPose.position.z = h

		#phi = math.atan2((self.wrist1ToObj.position.z - desiredPose.position.z), (self.wrist1ToObj.position.y - desiredPose.position.y))
		'''phi = math.atan2(self.wrist1ToObj.position.z, self.wrist1ToObj.position.y)
		quaternion = tf.transformations.quaternion_from_euler(phi,0,0)

		print "z and y"
		print self.wrist1ToObj.position.z
		print self.wrist1ToObj.position.y
		'''
		#print "quats"
		#print quaternion

		#print data.position.z
		#print desiredPose.position.z

		#print data.position.y
		#print desiredPose.position.y

		#print "z and y"
		#print data.position.z - desiredPose.position.z
		#print data.position.y - desiredPose.position.y

		#print "y and x"
		#print data.position.y
		#print data.position.x

		#print theta*180/pi
		#print phi*180/pi
		#print " "

		if onceFlag == False:
			# drive to position where r = 0.4 and h = 0.6
			jointStates = [-0.258, -0.098, -1.781, -1.262, -1.671, 1.57] # R1-R6
			self.execute_move(jointStates)
			onceFlag = True

		goal_jointStates = self.group.get_current_joint_values()
		#angle_to_go_t = theta - goal_jointStates[0]
		#angle_to_go_p = phi - goal_jointStates[3]



		'''print "actual:"
		#print goal_jointStates[0]*180/pi
		print goal_jointStates[3]*180/pi

		#print "to go:"
		#print angle_to_go_t*180/pi
		#print angle_to_go_p*180/pi

		print "goal"
		#print theta*180/pi
		print phi*180/pi
		'''

		#if phi < 0:
		#	phi = pi+phi
		#	print "phi < 0"
		#	print phi*180/pi

		print "move joint 1"
		self.move_joint_to_target(0, theta)
		print "move joint 4"

		phi = math.atan2(self.wrist1ToObj.position.x, self.wrist1ToObj.position.z)
		quaternion = tf.transformations.quaternion_from_euler(phi,0,0)

		print "x, y, z"
		print self.wrist1ToObj.position.x
		print self.wrist1ToObj.position.y
		print self.wrist1ToObj.position.z

		print "actual:"
		#print goal_jointStates[0]*180/pi
		print goal_jointStates[3]*180/pi

		#print "to go:"
		#print angle_to_go_t*180/pi
		#print angle_to_go_p*180/pi

		print "goal"
		#print theta*180/pi
		print phi*180/pi
		#phi = pi/2 - phi
		#print phi*180/pi

		print "vorschlag zielstate"
		print (goal_jointStates[3] - (pi/2 - phi))*180/pi

		self.move_joint_to_target(3, goal_jointStates[3] - (pi/2 - phi))#goal_jointStates[3] + phi)

		desiredPose = self.group.get_current_pose().pose

		desiredPose.orientation.x = quaternion[0]
		desiredPose.orientation.y = quaternion[1]
		desiredPose.orientation.z = quaternion[2]
		desiredPose.orientation.w = quaternion[3]

		#self.execute_move(desiredPose)
		rospy.sleep(2)

	# Publish the robot's trajectory
	def display_trajectory(self, plan):
		rospy.sleep(2)
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
		goal_jointStates[jointNr-1] = goal_jointStates[jointNr-1] + angleRad_inc

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

		if self.confirmation(goal):
			self.group.go(wait=True)
			self.group.clear_pose_targets()
			self.group.stop()

	# Move the robot along a specified way (plan)
	def execute_plan(self, plan):	#TODO
		# Retime all timestamps of the way-points to make the robot move at a specified speed
		plan = self.group.retime_trajectory(self.robot.get_current_state(), plan, self.speedScalingFactor)
		self.display_trajectory(plan)

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

		rospy.spin()

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
		jointNr = 1			# 1 to 6
		angleDeg_inc = 90
		ur5.move_joint(jointNr, angleDeg_inc)
		'''
		#ur5.go_home()
		

	except KeyboardInterrupt:
		rospy.signal_shutdown("KeyboardInterrupt")
		raise

if __name__ == '__main__':
	main(sys.argv)