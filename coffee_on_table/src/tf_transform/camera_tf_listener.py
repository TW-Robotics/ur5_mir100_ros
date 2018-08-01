#!/usr/bin/env python  
import roslib

import rospy
import math
import tf
import geometry_msgs.msg
import sys

def main(args):
	print "ino"
	rospy.init_node('camera_tf_listener')

	listener = tf.TransformListener()
	print "in"
	#rospy.wait_for_service('spawn')
	#spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
	#spawner(4, 2, 0, 'turtle2')

	#turtle_vel = rospy.Publisher('turtle2/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)

	# Do at a frequency of 10 Hz
	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		try:
			print "go"
			(trans,rot) = listener.lookupTransform('/camera_link', '/base_link', rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

		rate.sleep()
		print trans
		print rot
	print "done"

if __name__ == '__main__':
	main(sys.argv)