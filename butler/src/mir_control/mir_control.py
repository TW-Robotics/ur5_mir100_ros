#!/usr/bin/env python
import rospy
import sys
from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import Pose
from math import pi
import tf

class mirControler(object):
	def __init__(self):
		super(mirControler, self).__init__()

		rospy.Subscriber("/tf_objToOrigin", Pose, self.objToOrigin_callback, queue_size=1)
		rospy.Subscriber("/robot_pose", Pose, self.robotPose_callback, queue_size=1)
		self.pub = rospy.Publisher('move_base/goal', MoveBaseActionGoal, queue_size=10)

		rospy.sleep(1)		# Wait for publisher to be registered

		self.objToOrigin_pose = Pose()
		self.actPose = Pose()
		self.targetPose = Pose()

	def objToOrigin_callback(self, data):
		self.objToOrigin_pose = data

	def robotPose_callback(self, data):
		self.actPose = data

	def isAtGoal(self):
		thresholdP = 0.2
		thresholdO = 0.1
		if self.actPose.position.x < self.targetPose.position.x + thresholdP and self.actPose.position.x > self.targetPose.position.x - thresholdP:
			if self.actPose.position.y < self.targetPose.position.y + thresholdP and self.actPose.position.y > self.targetPose.position.y - thresholdP:
				if self.actPose.orientation.z < self.targetPose.orientation.z + thresholdO and self.actPose.orientation.z > self.targetPose.orientation.z - thresholdO:
					return True
		return False

	def moveToGoal(self, x, y, rz):
		rz = float(rz)*pi/180

		quats = tf.transformations.quaternion_from_euler(0, 0, rz)

		mirGoalMsg = MoveBaseActionGoal()
		#mirGoalMsg.header.stamp = rospy.Time.now()						# optional
		mirGoalMsg.header.frame_id = '/map' 							# Note: the frame_id must be map
		#mirGoalMsg.goal.target_pose.header.stamp = rospy.Time.now()	# optional
		mirGoalMsg.goal.target_pose.header.frame_id = '/map'			# Note: the frame_id must be map
		mirGoalMsg.goal.target_pose.pose.position.x = x
		mirGoalMsg.goal.target_pose.pose.position.y = y
		mirGoalMsg.goal.target_pose.pose.position.z = 0 				# z must be 0.0 (no height in the map)

		mirGoalMsg.goal.target_pose.pose.orientation.z = quats[2]
		mirGoalMsg.goal.target_pose.pose.orientation.w = quats[3]
		#mirGoalMsg.goal_id.stamp = rospy.Time.now()					# optional
		#mirGoalMsg.goal_id.id = '10'									# optional

		self.targetPose.position.x = float(x)
		self.targetPose.position.y = float(y)
		self.targetPose.orientation.z = float(quats[2])
		self.targetPose.orientation.w = float(quats[3])
		self.pub.publish(mirGoalMsg)
		#print quats[2]
		#print quats[3]
		print "Sent MiR to goal: x " + str(x) + ", y " + str(y) + ", rz " + str(rz)

if __name__ == '__main__':
	rospy.init_node('mirControl', anonymous=True)
	mir = mirControler()
	x = sys.argv[1]
	y = sys.argv[2]
	rz = sys.argv[3]
	mir.moveToGoal(x, y, rz)
	#print mir.isAtGoal()