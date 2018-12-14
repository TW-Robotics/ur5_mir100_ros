#!/usr/bin/env python  
import roslib

import rospy
import math
import tf
import geometry_msgs.msg
from geometry_msgs.msg import Pose
import sys
from math import pi

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

def listToPose1(trans, rot):
	quats = tf.transformations.quaternion_from_euler(0, 0, rot[2]*pi/180, 'rxyz')
	pose = geometry_msgs.msg.Pose()
	pose.position.x = trans[0]
	pose.position.y = trans[1]
	pose.position.z = trans[2]
	pose.orientation.x = quats[0]
	pose.orientation.y = quats[1]
	pose.orientation.z = quats[2]
	pose.orientation.w = quats[3]
	return pose

def main(args):
	# Init Node
	rospy.init_node('tf_listener_publisher')
	
	# Init Listener for tf-transformation
	listener = tf.TransformListener()

	# Init Publisher for camToBase Transformation
	objToBasePub = rospy.Publisher("/tf_objToBase", Pose, queue_size=1)
	objToWrist1Pub = rospy.Publisher("/tf_objToWrist1Pub", Pose, queue_size=1)
	objToOriginPub = rospy.Publisher("/tf_objToOrigin", Pose, queue_size=1)
	camToBasePub = rospy.Publisher("/tf_camToBase", Pose, queue_size=1)

	rospy.sleep(1)

	# Do at a frequency of 10 Hz
	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		#try:
			(trans, rot) = listener.lookupTransform('/base_footprint', '/obj_center_pose', rospy.Time(0))
			#(trans1, rot1) = listener.lookupTransform('/wrist_1_link', '/obj_center_pose', rospy.Time(0))
			#(trans2, rot2) = listener.lookupTransform('/map', '/obj_center_pose', rospy.Time(0))
			(trans3, rot3) = listener.lookupTransform('/camera_link', '/base_footprint', rospy.Time(0))
			#(trans, rot) = listener.lookupTransform('/camera_link', '/camera_color_frame', rospy.Time(0))
			objToBasePub.publish(listToPose(trans, rot))
			#objToWrist1Pub.publish(listToPose(trans1, rot1))
			#objToOriginPub.publish(listToPose(trans2, rot2))
			camToBasePub.publish(listToPose(trans3, rot3))
			print "test"
			print trans3
		#except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		#	rospy.loginfo("Warning!")
		#	continue
			rate.sleep()

if __name__ == '__main__':
	main(sys.argv)