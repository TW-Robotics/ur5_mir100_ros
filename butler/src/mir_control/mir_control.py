#!/usr/bin/env python
import rospy
import sys
from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import Pose
import math
import tf

''' Send the MiR Robot to a goal and check if it is already there '''

class mirControler():
	def __init__(self):
		# Init empty variables
		self.actPose = Pose()
		self.targetPose = Pose()

		# Init Subscriber and Publisher
		rospy.Subscriber("/robot_pose", Pose, self.robotPose_callback, queue_size=1)		# get actual pose of robot
		self.pub = rospy.Publisher('move_base/goal', MoveBaseActionGoal, queue_size=10)		# published pose are driven to

		# Wait for publisher to be registered
		rospy.sleep(1)

	def robotPose_callback(self, data):
		self.actPose = data

	# Check if robot is at goal within thresholds
	def isAtGoal(self, thresholdP, thresholdO):
		if math.fabs(self.actPose.position.x - self.targetPose.position.x) < thresholdP:
			if math.fabs(self.actPose.position.y - self.targetPose.position.y) < thresholdP:
				if math.fabs(self.actPose.orientation.z - self.targetPose.orientation.z) < thresholdO:
					return True
		return False

	# Send the MiR to a goal position
	def moveToGoal(self, x, y, rz):
		# Transform rz to radians
		rz = float(rz) * math.pi/180

		# Calculate orientation of robot in quaternions
		quats = tf.transformations.quaternion_from_euler(0, 0, rz)

		# Store the target position for isAtGoal-check
		self.targetPose.position.x = float(x)
		self.targetPose.position.y = float(y)
		self.targetPose.orientation.z = float(quats[2])
		self.targetPose.orientation.w = float(quats[3])

		# Build MiR-Message (lines which are commented out are not in MiR-Messages from the web interface)
		mirGoalMsg = MoveBaseActionGoal()
		#mirGoalMsg.header.stamp = rospy.Time.now()						# optional
		mirGoalMsg.header.frame_id = '/map' 							# Note: the frame_id must be map
		#mirGoalMsg.goal.target_pose.header.stamp = rospy.Time.now()	# optional
		mirGoalMsg.goal.target_pose.header.frame_id = '/map'			# Note: the frame_id must be map
		mirGoalMsg.goal.target_pose.pose.position.x = float(x)
		mirGoalMsg.goal.target_pose.pose.position.y = float(y)
		mirGoalMsg.goal.target_pose.pose.position.z = 0 				# z must be 0.0 (no height in the map)

		mirGoalMsg.goal.target_pose.pose.orientation.z = float(quats[2])
		mirGoalMsg.goal.target_pose.pose.orientation.w = float(quats[3])
		#mirGoalMsg.goal_id.stamp = rospy.Time.now()					# optional
		#mirGoalMsg.goal_id.id = '10'									# optional
		
		# Publish Message -> Send MiR to goal
		self.pub.publish(mirGoalMsg)
		print "Sent MiR to goal: x " + str(x) + ", y " + str(y) + ", rz " + str(rz)

if __name__ == '__main__':
	rospy.init_node('mirControl', anonymous=True, disable_signals=True)
	mir = mirControler()
	x = sys.argv[1]
	y = sys.argv[2]
	rz = sys.argv[3]
	mir.moveToGoal(x, y, rz)
