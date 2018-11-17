#!/usr/bin/env python  
import roslib

import rospy
import tf
import sys

from geometry_msgs.msg import Pose

obj_center_pose = Pose()
obj_center_pose.orientation.w = 1

def obj_pose_callback(data):
	global obj_center_pose
	obj_center_pose = data

def main(args):
	rospy.init_node('tf_broadcaster')

	rospy.Subscriber("/tf_objToCam", Pose, obj_pose_callback, queue_size=1)	# Subscribe to xyz-Position of object

	br = tf.TransformBroadcaster()

	# Do at a frequency of 10 Hz
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		br.sendTransform((obj_center_pose.position.y, -obj_center_pose.position.x, obj_center_pose.position.z),
						 (obj_center_pose.orientation.x, obj_center_pose.orientation.y, obj_center_pose.orientation.z, obj_center_pose.orientation.w),
						 rospy.Time.now(),
						 "obj_center_pose",
						 "camera_aligned_depth_to_color_frame")
		print obj_center_pose
		rate.sleep()

if __name__ == '__main__':
    main(sys.argv)