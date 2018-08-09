#!/usr/bin/env python  
import roslib

import rospy
import math
import tf
import geometry_msgs.msg
from geometry_msgs.msg import Pose
import sys

# Convert lists to pose-obect so only 1 object has to be published
def listToPose(trans, rot):
	pose = geometry_msgs.msg.Pose()
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
	rospy.init_node('object_tf_listener')
	
	# Init Listener for tf-transformation
	listener = tf.TransformListener()

	# Init Publisher for camToBase Transformation
	objToBasePub = rospy.Publisher("/tf_objToBase", Pose, queue_size=1)

	# Do at a frequency of 10 Hz
	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		try:
			(trans, rot) = listener.lookupTransform('/base_link', '/obj_center_pose', rospy.Time(0))
			#(trans, rot) = listener.lookupTransform('/camera_link', '/camera_color_frame', rospy.Time(0))
			objToBasePub.publish(listToPose(trans, rot))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

		rate.sleep()

if __name__ == '__main__':
	main(sys.argv)