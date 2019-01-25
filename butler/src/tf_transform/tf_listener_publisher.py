#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import Pose
import sys
from math import pi

''' Listen to transformation between object and base of robot
	to make it turn correctly to object while searching it'''

# Convert lists to pose-obect so a standard pose message can be published
def listToPose(trans, rot):
	pose = Pose()
	pose.position.x = trans[0]
	pose.position.y = trans[1]
	pose.position.z = trans[2]
	pose.orientation.x = rot[0]
	pose.orientation.y = rot[1]
	pose.orientation.z = rot[2]
	pose.orientation.w = rot[3]
	return pose

def main(args):
	# Init Node
	rospy.init_node('tf_listener_publisher', disable_signals=True)
	
	# Init Listener for tf-transformation
	listener = tf.TransformListener()

	# Init Publishers
	objToBasePub = rospy.Publisher("/tf_objToBase", Pose, queue_size=1)		# For movement of R1-axis of UR when searching object
	rospy.sleep(1)

	# Do at a frequency of 10 Hz
	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		#try:
			# Get transformation and publish it rearranged to a list
			(trans, rot) = listener.lookupTransform('/base_footprint', '/obj_center_pose', rospy.Time(0))
			objToBasePub.publish(listToPose(trans, rot))
		#except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		#	rospy.loginfo("Warning!")
		#	continue
			rate.sleep()

if __name__ == '__main__':
	main(sys.argv)