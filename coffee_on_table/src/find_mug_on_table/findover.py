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
import ur5_control
import math
from math import pi

# for publishing coordinate frame
import tf

''' Class to subscribe to
		- Output-Image from YOLOnet with bounding boxes
		- Output-Array of bounding boxes (Positions of edges)
		- Kamera-Image where the depth-image is aligned to the color-image
		- Kamera-Calibration data
	Calculates the position of the center of the innerClass in the outerClass(e.g. center of cup on table)'''
class rossinator(object):
	# Initialize CVBridge
	bridge = CvBridge()
	# Initialize class-variable to store bounding boxes
	currentBoundingBoxes = BoundingBoxes()

	coc = Point() 	# Center of cup in image coordinates (px)
	obj_radius = 0
	is_approximation = True

	onlyOneClass = False
	innerClass = ""
	outerClass = ""
	strictness = ""

	def __init__(self):
		# Get and validate arguments
		if len(sys.argv) == 2:
			self.onlyOneClass = True
			self.innerClass = sys.argv[1]
		elif len(sys.argv) < 3:
			print("  ERROR: Too few arguments given!")
			self.onlyOneClass = True
			self.innerClass = "cup"
			#return
		if self.onlyOneClass == False:
			self.innerClass = sys.argv[1]
			self.outerClass = sys.argv[2]
			self.strictness = sys.argv[3]

		# Initialize Publisher and Subscribers
		self.mypub = rospy.Publisher("/roi", CompressedImage, queue_size=1)	#TODO Not used yet
		self.object_pos_pub = rospy.Publisher("/tf_objToCam", Pose, queue_size=1)	# Publish xyz-Position of object
		rospy.Subscriber("/camera/color/image_raw_rotated", Image, self.image_callback, queue_size=1)				# YOLOnet Output-Image
		rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.bounding_callback, queue_size=1)	# Bounding-Box-Array
		rospy.Subscriber("/camera/aligned_depth_to_color/image_raw_rotated", Image, self.depth_callback, queue_size=1)	# Depth-Image aligned to Color-Image
		rospy.Subscriber("/camera/aligned_depth_to_color/camera_info", CameraInfo, self.cameraInfo_callback, queue_size=1)		# Camera Calibration
		rospy.Subscriber("/tf_camToBase", Pose, self.camPose_callback, queue_size=1)		# Camera Position and Orientation (dependent on robot pose)

		#self.cup_size_middler = 0
		#self.cup_size_middler_count = 0

	# Publish Pose of object
	def pub_object_pose(self, xyz, rxyzw):
		pubPose = Pose()
		pubPose.position.x = xyz.x / 1000
		pubPose.position.y = xyz.y / 1000
		pubPose.position.z = xyz.z / 1000
		pubPose.orientation.x = rxyzw[0]
		pubPose.orientation.y = rxyzw[1]
		pubPose.orientation.z = rxyzw[2]
		pubPose.orientation.w = rxyzw[3]
		#print pubPose
		self.object_pos_pub.publish(pubPose)

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
		# TODO COMMENT!!!!
		#self.inner_in_outer()
		#if self.inner_in_outer():
		#	a = 1#print('Found ' + self.innerClass + ' ' + self.strictness + ' ' + self.outerClass)
		#else:
			#ur5_control.searchObject()

	# Write bounding boxes to class-variable
	def bounding_callback(self, data):
		self.currentBoundingBoxes = data

	# Calculate depth-image
	def depth_callback(self, data):
		try:
			# Convert from ROS-Image to CV2::Mat
			self.cv_depth_image = self.bridge.imgmsg_to_cv2(data,"32FC1")
			#print self.cv_depth_image
			# Convert from CV2::Mat to Array with depth-Values
			self.depth_array = np.array(self.cv_depth_image, dtype=np.float32)
			#self.find_handle()
		except CvBridgeError as e:
			print(e)

	# Calculate and publish pose where to grab the cup
	def find_handle(self, threshold):
		self.coc.x = int(self.coc.x)
		self.coc.y = int(self.coc.y)
		self.obj_radius = int(self.obj_radius)
		# Set all pixels to 255, which are farer away then "threshold" (depends on robot pre-position)
		# TODO: Write function which calculates best threshold by it's own with percent of white/black pixels 
		cv2.threshold(self.cv_depth_image, threshold, 255, cv2.THRESH_BINARY, self.cv_depth_image)
		print threshold
		# Convert the image to uint8 to make it processible with other functions
		self.cv_depth_image = np.uint8(self.cv_depth_image)
		# Blur the image to lose small regions which could be detected wrong
		self.cv_depth_image = cv2.medianBlur(self.cv_depth_image, 5)
		
		# Find circles - currently not used
		'''circles = cv2.HoughCircles(self.cv_depth_image,cv2.HOUGH_GRADIENT,1,30,
                            param1=50,param2=30,minRadius=20,maxRadius=0)
		circles = np.uint16(np.around(circles))
		print circles
		maxR = 0
		biggestCircle = circles[0]
		for i in circles[0,:]:
			if i[2] > maxR:
				biggestCircle = i

		i = biggestCircle
		'''

		# Calculate the area, where around the center of the cup, where should be searched for the handle
		# TODO: Don't search in area of the cup center
		sizeFactor = 1.5 
		start_y = int(self.coc.x - sizeFactor*self.obj_radius)
		start_x = int(self.coc.y - sizeFactor*self.obj_radius)
		end_y = int(self.coc.x + sizeFactor*self.obj_radius)
		end_x = int(self.coc.y + sizeFactor*self.obj_radius)
		if start_y < 0:
			start_y = 0
		if start_x < 0:
			start_x = 0
		if end_y >= len(self.cv_depth_image[1]):
			end_y = len(self.cv_depth_image[1])-1
		if end_x >= len(self.cv_depth_image):
			end_x = len(self.cv_depth_image)-1

		# Calculate the distance of every pixel which is black to the center of the cup
		# TODO optimize function to be quicker
		biggestDist = 0.0
		biggestDistPos_px = Point()
		for x in range(start_x, end_x):
			for y in range(start_y, end_y):
				if self.cv_depth_image[x][y] == 0:
					dist = math.sqrt((self.coc.x-y)**2 + (self.coc.y-x)**2)
					if dist > biggestDist:
						biggestDist = dist
						biggestDistPos_px.x = y
						biggestDistPos_px.y = x
		# Store Point of Interest
		poi = biggestDistPos_px
		poi.x = int(poi.x)
		poi.y = int(poi.y)

		#self.cv_rgb_image = self.cv_depth_image

		# Calculate grab-point and grab-pose
		grabPoint = Point()
		if poi.x != 0:		# if a poi has been found
			# Vector from poi to coc
			v = Point()
			v.x = self.coc.x - poi.x
			v.y = self.coc.y - poi.y
			# Length of vector v
			lv = math.sqrt((self.coc.x - poi.x)**2 + (self.coc.y - poi.y)**2)
			# Resize vector to unit vector
			v.x = v.x / lv
			v.y = v.y / lv

			# Grab-Point is in the middle of the poi and where v intersects with the radius
			grabPoint.x = int(poi.x + v.x * (lv-self.obj_radius)/2)
			grabPoint.y = int(poi.y + v.y * (lv-self.obj_radius)/2)
			grabPoint.z = self.depth_array[grabPoint.y, grabPoint.x]
			# print grabPoint

			# Calculation of angle alpha of vecotr v to axis
			# Make a vector along the x-axis
			x_axis = Point()
			x_axis.x = 1
			x_axis.y = 0
			lx_axis = 1
			lv = 1	# Set length to 1 because v is already resized
			dotProduct = v.x*x_axis.x + v.y*x_axis.y
			alpha = math.acos(dotProduct/(lv*lx_axis))

			# Detect if angle is positive or negative
			if poi.y > self.coc.y:
				alpha = -alpha
			print alpha*180/pi
			# Greifbar von 0 bis 180 grad, wenn letztes Gelenk auf 90 grad gedreht ist
			# TODO Calculate angle in case it is bigger than 90 deg (grabable?)
			if alpha < 0:
				alpha = pi + alpha # + = - -alpha
				print "Corrected to " + str(alpha*180/pi)

			quats = tf.transformations.quaternion_from_euler(alpha-pi/2, -pi/2, pi/2, 'rzyx')
			# TOOL0quats = tf.transformations.quaternion_from_euler(0, 0, alpha, 'rxyz')
			self.alpha = alpha
			#quats = tf.transformations.quaternion_from_euler(0, -pi/2, pi/2+alpha, 'rzyx')
			#quats = tf.transformations.quaternion_from_euler(pi/2, alpha, 0, 'rxyz')
			#print quats

			# Draw grab-point
			cv2.circle(self.cv_rgb_image ,(grabPoint.x, grabPoint.y),2,(0,150,150),3)

		# Draw circle for border of cup
		cv2.circle(self.cv_rgb_image ,(self.coc.x,self.coc.y),self.obj_radius,(0,255,0),2)
		# Draw circle for center of cup
		cv2.circle(self.cv_rgb_image ,(self.coc.x,self.coc.y),2,(0,0,255),3)

		# Draw circles and lines for area of interest
		cv2.circle(self.cv_rgb_image ,(start_y, start_x),2,(0,255,0),3)
		cv2.circle(self.cv_rgb_image ,(start_y, end_x),2,(0,255,0),3)
		cv2.circle(self.cv_rgb_image ,(end_y, start_x),2,(0,255,0),3)
		cv2.circle(self.cv_rgb_image ,(end_y, end_x),2,(0,255,0),3)
		cv2.line(self.cv_rgb_image, (start_y, start_x), (end_y, start_x), (150,150,0))
		cv2.line(self.cv_rgb_image, (start_y, start_x), (start_y, end_x), (150,150,0))
		cv2.line(self.cv_rgb_image, (end_y, end_x), (start_y, end_x), (150,150,0))
		cv2.line(self.cv_rgb_image, (end_y, end_x), (end_y, start_x), (150,150,0))

		# Draw circle for point of interest
		cv2.circle(self.cv_rgb_image ,(poi.x, poi.y),2,(0,0,255),3)
		# Draw line from poi to center of cup
		cv2.line(self.cv_rgb_image, (self.coc.x, self.coc.y), (poi.x, poi.y), (150,150,0))

		# Display the images
		cv2.imshow("CutOff", self.cv_depth_image)
		#cv2.waitKey(0)

		#inp = raw_input("Press something for second image: ")[0]
		cv2.imshow("Circles", self.cv_rgb_image)
		cv2.waitKey(1)

		inp = raw_input("point correct? y/n: ")[0]

		if grabPoint.z != 0 and inp == 'y':
			grabPoint = self.calculate_center_coordinates(grabPoint.x, grabPoint.y, grabPoint.z)
			self.pub_object_pose(grabPoint, quats)
			print "Grab-Point z: " + str(grabPoint.z)
			return True
		return False


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
							#print "APPROXIMATION: "
							self.is_approximation = True
							depth = self.distance_to_object_approximator(inner_box.xmax - inner_box.xmin)
						self.is_approximation = False
						self.obj_center_pos = self.calculate_center_coordinates(center_x, center_y, depth)
						self.pub_object_pose(self.obj_center_pos, [0, 0, 0, 1])

						#print "Center of box in mm (x, y, z): {0:3.0f}, {1:3.0f}, {2:3.0f}".format(x, y, z)
						self.obj_radius = (inner_box.xmax-inner_box.xmin)/2
						self.coc.x = center_x
						self.coc.y = center_y

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
		#cv2.namedWindow(windowName)
		#cv2.moveWindow(windowName, 800, 0)
		#cv2.imshow(windowName, self.cv_depth_image)
		#cv2.waitKey(1)

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
		center_point = Point()
		center_point.x = x
		center_point.y = y
		center_point.z = z
		return center_point

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

	#cupOnTable.find_handle()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print ("Shutting down ROS Cupfinder")

	# Close all Image-Windows 
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
