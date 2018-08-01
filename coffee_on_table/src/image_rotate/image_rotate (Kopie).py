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
	
	def __init__(self):
		# Initialize Publisher and Subscribers
		self.mypub = rospy.Publisher("/camera/color/image_raw_rotated",Image, queue_size=1)
		rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback, queue_size=1)				# YOLOnet Output-Image
	
	def image_callback(self, data):
		try:
			# Convert from ROS-Image to CV2::Mat
			cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
		except CvBridgeError as e:
			print(e)

		rows = cv_image.shape[0]
		cols = cv_image.shape[1]
		print(rows)
		M = cv2.getRotationMatrix2D((cols/2,rows/2),180,1)
		dst = cv2.warpAffine(cv_image,M,(cols,rows))

		self.mypub.publish(self.bridge.cv2_to_imgmsg(dst,"bgr8"))

def main(args):
	# Initialize ros-node and Class
	rospy.init_node('camera_rotate', anonymous=True)
	cupOnTable = rossinator()
	
	try:
	    rospy.spin()
	except KeyboardInterrupt:
		print ("Shutting down ROS Cupfinder")

	# Close all Image-Windows 
	cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)