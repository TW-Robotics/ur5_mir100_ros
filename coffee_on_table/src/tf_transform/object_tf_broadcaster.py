#!/usr/bin/env python  
import roslib

import rospy
import tf
import sys

from geometry_msgs.msg import Point

obj_center_point = Point()

def obj_pos_callback(data):
	global obj_center_point
	obj_center_point = data

def main(args):
	rospy.init_node('object_tf_broadcaster')

	rospy.Subscriber("/tf_objToCam", Point, obj_pos_callback, queue_size=1)	# Subscribe to xyz-Position of object

	br = tf.TransformBroadcaster()
	global ob

	# Do at a frequency of 10 Hz
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		br.sendTransform((obj_center_point.x, obj_center_point.y, obj_center_point.z),
						 (0.0, 0.0, 0.0, 1.0),
						 rospy.Time.now(),
						 "obj_center_pose",
						 "camera_aligned_depth_to_color_frame")
		print obj_center_point
		rate.sleep()

if __name__ == '__main__':
    main(sys.argv)