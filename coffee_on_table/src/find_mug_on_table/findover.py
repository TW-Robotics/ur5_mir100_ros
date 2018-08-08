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
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
import sensor_msgs.point_cloud2 as pc2

# for publishing coordinate frame
import tf

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

	onlyOneClass = False
	innerClass = ""
	outerClass = ""
	strictness = ""

	def __init__(self):
		# Get and validate arguments
		if len(sys.argv) == 2:
			self.onlyOneClass = True
		elif len(sys.argv) < 3:
			print("  ERROR: Too few arguments given!")
			return
		self.innerClass = sys.argv[1]
		if self.onlyOneClass == False:
			self.outerClass = sys.argv[2]
			self.strictness = sys.argv[3]

		# Initialize Publisher and Subscribers
		self.mypub = rospy.Publisher("/roi", CompressedImage, queue_size=1)	#TODO Not used yet
		self.object_pos_pub = rospy.Publisher("/tf_objToCam", Point, queue_size=1)	# Publish xyz-Position of object
		rospy.Subscriber("/camera/color/image_raw_rotated", Image, self.image_callback, queue_size=1)				# YOLOnet Output-Image
		rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.bounding_callback, queue_size=1)	# Bounding-Box-Array
		rospy.Subscriber("/camera/aligned_depth_to_color/image_raw_rotated", Image, self.depth_callback, queue_size=1)	# Depth-Image aligned to Color-Image
		rospy.Subscriber("/camera/aligned_depth_to_color/camera_info", CameraInfo, self.cameraInfo_callback, queue_size=1)		# Camera Calibration
		rospy.Subscriber("/tf_camToBase", Pose, self.camPose_callback, queue_size=1)		# Camera Position and Orientation (dependent on robot pose)

		#self.cup_size_middler = 0
		#self.cup_size_middler_count = 0

	# Get the pose of the camera dependent on the robot pose
	def camPose_callback(self, data):
		self.camPose = data

	# Get camera-info and make it accesible in the class
	def cameraInfo_callback(self, data):
		self.camInfo = data

	# Check if there is a innerClass in an outerClass
	def image_callback(self, data):
		try:
			# Convert from ROS-Image to CV2::Mat
			self.cv_rgb_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
		except CvBridgeError as e:
			print(e)

		# Check if there is a inner class in an outer class in the image and display message
		if self.inner_in_outer():
			a = 1#print('Found ' + self.innerClass + ' ' + self.strictness + ' ' + self.outerClass)



	# Write bounding boxes to class-variable
	def bounding_callback(self, data):
		self.currentBoundingBoxes = data

	# Calculate depth-image
	def depth_callback(self, data):
		try:
			# Convert from ROS-Image to CV2::Mat
			self.cv_depth_image = self.bridge.imgmsg_to_cv2(data,"32FC1")
			# Convert from CV2::Mat to Array with depth-Values
			self.depth_array = np.array(self.cv_depth_image, dtype=np.float32)
		except CvBridgeError as e:
			print(e)

	def inner_in_outer(self):
		foundInnerInOuter = False

		# Check for box in box
		for outer_box in self.currentBoundingBoxes.bounding_boxes:
			if outer_box.Class == self.outerClass or self.onlyOneClass == True:
				for inner_box in self.currentBoundingBoxes.bounding_boxes:
					if inner_box.Class == self.innerClass and self.box_is_in_box(outer_box, inner_box):
						center_x = inner_box.xmin+(inner_box.xmax-inner_box.xmin)/2
						center_y = inner_box.ymin+(inner_box.ymax-inner_box.ymin)/2
						depth = self.depth_array[center_y][center_x]
						# print("center is: " + str(center_y) + ", " + str(center_x) + " Depth: " + str(self.depth))
						# Draw circle on center of bounding box of inner class
						cv2.circle(self.cv_depth_image,(center_x, center_y), 10, (0,0,0), -1)
						foundInnerInOuter = True

						# Caluclate coordinates in mm (Center of image is zero)
						if depth == 0:		# approximate if there is no depth-data
							print "APPROXIMATION: "
							depth = self.distance_to_object_approximator(inner_box.xmax - inner_box.xmin)
						[x, y, z] = self.calculate_center_coordinates(center_x, center_y, depth)
						obj_center_point = Point()
						obj_center_point.x = x / 1000
						obj_center_point.y = y / 1000
						obj_center_point.z = z / 1000
						self.object_pos_pub.publish(obj_center_point)
						print "Center of box in mm (x, y, z): {0:3.0f}, {1:3.0f}, {2:3.0f}".format(x, y, z)
						
						# Old + Debug
						#else:
							#print("Object is too far away. Come closer to get coordinates")

						#cropped_encoded_img = self.cv_rgb_image[inner_box.ymin:inner_box.ymax,inner_box.xmin:inner_box.xmax]
						#self.find_cup(cropped_encoded_img)						
						# For measurement of approximation-parameters
						#if self.cup_size_middler_count < 50:
						#	self.cup_size_middler_count = self.cup_size_middler_count+1
						#	self.cup_size_middler = self.cup_size_middler + inner_box.xmax-inner_box.xmin

						#if self.cup_size_middler_count == 50:
						#	print self.cup_size_middler/50
						#	self.cup_size_middler = 0
						#	self.cup_size_middler_count = 0

						#print inner_box.xmax-inner_box.xmin

		# Display depth-array with drawn circle
		windowName = "Depth image with circle"
		cv2.namedWindow(windowName)
		cv2.moveWindow(windowName, 800, 0)
		cv2.imshow(windowName, self.cv_depth_image)
		cv2.waitKey(1)

		# Return True if found, otherwise False
		if foundInnerInOuter == True:
			return True
		return False

	# Interpolation function based on real measurements
	def distance_to_object_approximator(self, px_width):
		# Parameters can be found in excel-sheet. Times 10 to calculate distance in mm
		return 5474.6*(px_width**-1.015)*10

	# Calculate coordinates in mm (Center of image is zero)
	def calculate_center_coordinates(self, center_x, center_y, z):
		data = self.camInfo
		K = [[data.K[0], data.K[1], data.K[2]], [data.K[3], data.K[4], data.K[5]], [data.K[6], data.K[7], data.K[8]]]
		[x, y, z] = np.dot(np.linalg.inv(K), [center_x*z, center_y*z, z])
		return [x, y, z]

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

		if self.strictness == 'outside' or self.onlyOneClass == True:
			return True

		return False

	def find_cup(self, cropped_encoded_img):

		grey_img = cv2.cvtColor(cropped_encoded_img,cv2.COLOR_BGR2GRAY)
		ret, thresh = cv2.threshold(grey_img,127,255,0)
		_, contours, h = cv2.findContours(thresh,1,2)
		largest = None
		for contour in contours:
			approx = cv2.approxPolyDP(contour,0.01*cv2.arcLength(contour,True),True)
			if len(approx) == 4:
				if largest is None or cv2.contourArea(contour) > cv2.contourArea(largest):
					largest = contour

		if largest != None:
			cv2.drawContours(cropped_encoded_img,[largest],0,(0,0,255),3)
		cv2.imshow('puppy', cropped_encoded_img)
		cv2.waitKey(1)

def main(args):
	# Initialize ros-node and Class
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
