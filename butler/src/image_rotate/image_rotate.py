#!/usr/bin/env python
# Python libs
import sys, time

# numpy and scipy
import numpy as np

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy

# cv_bridge - needed to convert between ROS Image Msg and OpenCV cv::Mat
from cv_bridge import CvBridge, CvBridgeError

# Ros Messages
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

class rossinator:
	# Initialize CVBridge
	bridge = CvBridge()
	angle = 180			# Standard 180 degrees - reset if param is given

	def __init__(self):
		# Get and validate parameter
		if rospy.has_param('camera_rotate/rotation_angle'):
			self.angle = rospy.get_param('camera_rotate/rotation_angle')

		# Initialize Publisher and Subscribers
		self.colorImgPub = rospy.Publisher("/camera/color/image_raw_rotated",Image, queue_size=1)
		rospy.Subscriber("/camera/color/image_raw", Image, self.color_image_callback, queue_size=1)
		self.depthImgPub = rospy.Publisher("/camera/aligned_depth_to_color/image_raw_rotated",Image, queue_size=1)
		rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_image_callback, queue_size=1)
	
	def color_image_callback(self, data):
		#cv2.imshow("CutOff", self.bridge.imgmsg_to_cv2(data, "bgr8"))
		#cv2.waitKey(1)
		#rospy.rostime.wallsleep(0.1)
		image = self.rotate_image(data, "bgr8")
		self.colorImgPub.publish(self.bridge.cv2_to_imgmsg(image,"bgr8"))

	def depth_image_callback(self, data):
		#cv2.imshow("IMG", self.bridge.imgmsg_to_cv2(data, "32FC1"))
		#cv2.waitKey(1)
		#rospy.rostime.wallsleep(0.1)
		image = self.rotate_image(data, "32FC1")
		self.depthImgPub.publish(self.bridge.cv2_to_imgmsg(image,"32FC1"))

	def rotate_image(self, data, imgType):
		try:
			# Convert from ROS-Image to CV2::Mat
			cv_image = self.bridge.imgmsg_to_cv2(data, imgType)
		except CvBridgeError as e:
			print(e)

		# Rotate image
		rows = cv_image.shape[0]
		cols = cv_image.shape[1]
		M = cv2.getRotationMatrix2D((cols/2,rows/2), float(self.angle), 1)
		return cv2.warpAffine(cv_image,M,(cols,rows))


def main(args):
	# Initialize ros-node and Class
	rospy.init_node('camera_rotate', anonymous=True, disable_signals=True)
	rossinator()

	try:
	    rospy.spin()
	except KeyboardInterrupt:
		print ("Shutting down Image-Rotater")

	# Close all Image-Windows 
	cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)