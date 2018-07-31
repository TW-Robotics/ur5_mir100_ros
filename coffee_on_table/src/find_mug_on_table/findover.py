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
from std_msgs.msg import String
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

''' Class to subscribe to
		- Output-Image from YOLOnet with bounding boxes
		- Output-Array of bounding boxes (Positions of edges)
		- Kamera-Image where the depth-image is aligned to the color-image
		- Kamera-Calibration data
	Calculates the position of the center of the innerClass in the outerClass(e.g. center of cup on table)'''
class rossinator:
	# Initialize CVBridge
	bridge = CvBridge()
	# Initialize class-variable to store bounding boxes
	currentBoundingBoxes = BoundingBoxes()
	outerClass = sys.argv[1]
	innerClass = sys.argv[2]
	strictness = sys.argv[3]

	def __init__(self):
		# Initialize Publisher and Subscribers
		mypub = rospy.Publisher("/roi",CompressedImage, queue_size=1)	#TODO Not used yet
		rospy.Subscriber("/darknet_ros/detection_image", Image, self.image_callback, queue_size=1)				# YOLOnet Output-Image
		rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.bounding_callback, queue_size=1)	# Bounding-Box-Array
		rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_callback, queue_size=1)	# Depth-Image aligned to Color-Image
		rospy.Subscriber("/camera/aligned_depth_to_color/camera_info", CameraInfo, self.cameraInfo_callback, queue_size=1)		# Camera Calibration

	# Get camera-info and make it accesible in the class
	def cameraInfo_callback(self, data):
		self.camInfo = data

	# Check if there is a innerClass in an outerClass
	def image_callback(self, data):
		'''try:
			# Convert from ROS-Image to CV2::Mat
			cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
		except CvBridgeError as e:
			print(e)
		windowName = "Image from YOLOnet"
		cv2.namedWindow(windowName)
		cv2.moveWindow(windowName, 800, 500)
		cv2.imshow(windowName, cv_image)
		cv2.waitKey(1)'''

		# Check if there is a inner class in an outer class in the image and display message
		if self.inner_in_outer():
			print('Found ' + self.innerClass + ' ' + self.strictness + ' ' + self.outerClass)

	# Write bounding boxes to class-variable
	def bounding_callback(self, data):
		self.currentBoundingBoxes = data

	# Calculate depth-image
	def depth_callback(self, data):
		try:
			# Convert from ROS-Image to CV2::Mat
			self.cv_image = self.bridge.imgmsg_to_cv2(data,"32FC1")
			# Convert from CV2::Mat to Array with depth-Values
			self.depth_array = np.array(self.cv_image, dtype=np.float32)
		except CvBridgeError as e:
			print(e)

	def inner_in_outer(self):
		foundInnerInOuter = False

		# Check for box in box
		for outer_box in self.currentBoundingBoxes.bounding_boxes:
			if outer_box.Class == self.outerClass:
				for inner_box in self.currentBoundingBoxes.bounding_boxes:
					if inner_box.Class == self.innerClass and self.box_is_in_box(outer_box, inner_box):
						center_x = inner_box.xmin+(inner_box.xmax-inner_box.xmin)/2
						center_y = inner_box.ymin+(inner_box.ymax-inner_box.ymin)/2
						depth = self.depth_array[center_y][center_x]
						# print("center is: " + str(center_y) + ", " + str(center_x) + " Depth: " + str(self.depth))
						# Draw circle on center of bounding box of inner class
						cv2.circle(self.cv_image,(center_x, center_y), 10, (0,0,0), -1)
						foundInnerInOuter = True

						# Calculate coordinates in mm (Center of image is zero)
						data = self.camInfo
						z = depth
						if z != 0:
							K = [[data.K[0], data.K[1], data.K[2]], [data.K[3], data.K[4], data.K[5]], [data.K[6], data.K[7], data.K[8]]]
							[x, y, z] = np.dot(np.linalg.inv(K), [center_x*z, center_y*z, z])
							print "Center of box in mm (x, y, z): {0:3.0f}, {1:3.0f}, {2:3.0f}".format(x, y, z)
						else:
							print("Object is too far away. Come closer to get coordinates")

		# Display depth-array with drawn circle
		windowName = "Depth image with circle"
		cv2.namedWindow(windowName)
		cv2.moveWindow(windowName, 800, 0)
		cv2.imshow(windowName, self.cv_image)
		cv2.waitKey(1)

		# Return True if found, otherwise False
		if foundInnerInOuter == True:
			return True
		return False

	# Return True if inner box is in outer box (accroding to strictness), otherwise False
	def box_is_in_box(self, outer_box, inner_box):
		if self.strictness == 'inside':
			if inner_box.xmin > outer_box.xmin and inner_box.xmax < outer_box.xmax and inner_box.ymin > outer_box.ymin and inner_box.ymax < outer_box.ymax:
				return True

		if self.strictness == 'touching':
			center_x = inner_box.xmin+(inner_box.xmax-inner_box.xmin)/2
			center_y = inner_box.ymin+(inner_box.ymax-inner_box.ymin)/2
			if center_x > outer_box.xmin and center_x < outer_box.xmax and center_y > outer_box.ymin and center_y < outer_box.ymax:
				return True

		if self.strictness == 'outside':
			return True

		return False

def main(args):
	if len(sys.argv) < 3:
		print("  ERROR: Too few arguments given!")
		return

	#Initializes and cleanup ros node
	rospy.init_node('cupfinder', anonymous=True)
	cupOnTable = rossinator()
	print('Searching for ' + cupOnTable.innerClass + ' ' + cupOnTable.strictness + ' ' + cupOnTable.outerClass)

	try:
	    rospy.spin()
	except KeyboardInterrupt:
		print ("Shutting down ROS Cupfinder")

	# Close all Image-Windows 
	cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
