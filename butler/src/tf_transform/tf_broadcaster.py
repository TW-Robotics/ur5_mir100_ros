#!/usr/bin/env python  
import rospy
import tf
import sys
from geometry_msgs.msg import Pose

''' Subscribe to transformation between object and camera and 
	broadcast it to tf so other transformations can be calculated'''

obj_center_pose = Pose()
obj_center_pose.orientation.w = 1 	# Init as w=1 to make sure pose is valid

def obj_pose_callback(data):
	global obj_center_pose
	obj_center_pose = data

def main(args):
	# Init Node
	rospy.init_node('tf_broadcaster', disable_signals=True)

	# Init subscriber to Pose of object relative to camera
	rospy.Subscriber("/tf_objToCam", Pose, obj_pose_callback, queue_size=1)

	# Init tf-broadcaster to forward pose to tf
	br = tf.TransformBroadcaster()

	# Do at a frequency of 10 Hz
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		# Send transformation to tf
		br.sendTransform((obj_center_pose.position.y, -obj_center_pose.position.x, obj_center_pose.position.z),
						 (obj_center_pose.orientation.x, obj_center_pose.orientation.y, obj_center_pose.orientation.z, obj_center_pose.orientation.w),
						 rospy.Time.now(),
						 "obj_center_pose",
						 "camera_aligned_depth_to_color_frame")
		rate.sleep()

if __name__ == '__main__':
    main(sys.argv)