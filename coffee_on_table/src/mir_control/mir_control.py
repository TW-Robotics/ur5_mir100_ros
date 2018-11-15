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
		pub = rospy.Publisher('move_base/goal', MoveBaseActionGoal, queue_size=10)

		self.objToOrigin_pose = Pose()

	def objToOrigin_callback(self, data):
		self.objToOrigin_pose = data

	def moveToGoal(x, y, rz):
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

		rospy.sleep(1)		# Wait for publisher to be registered
		self.pub.publish(mirGoalMsg)
		#print quats[2]
		#print quats[3]
		print "Sent MiR to goal: x " + str(x) + ", y " + str(y) + ", rz " + str(rz) + "\n"

if __name__ == '__main__':
	rospy.init_node('mirControl', anonymous=True)
	x = sys.argv[1]
	y = sys.argv[2]
	rz = sys.argv[3]
	moveToGoal(x, y, rz)