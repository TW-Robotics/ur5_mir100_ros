#!/usr/bin/env python
import sys, time
import numpy as np

# Ros libraries
import roslib
import rospy

# OpenCV
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Ros Messages
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point

import ur5_control
import math
from math import pi
import tf

''' Calculate and publish pose of object and grasping points '''

class img_processing():
	def __init__(self, objectToSearch):
		# Initialize CVBridge
		self.bridge = CvBridge()

		# Initialize class-variables
		self.currentBoundingBoxes = BoundingBoxes()
		self.objCenterPX = Point()			# Center of object in image coordinates (px)
		self.objCenterM = Point()
		self.obj_radiusPX = 0  				# Radius of object in image coordinates (px)
		self.objectClass = objectToSearch
		self.camInfo = CameraInfo()

		# Initialize Publisher and Subscribers
		self.object_pos_pub = rospy.Publisher("/tf_objToCam", Pose, queue_size=1)											# Publish xyz-Position of object
		rospy.Subscriber("/camera/color/image_raw_rotated", Image, self.image_callback, queue_size=1)						# YOLOnet Output-Image
		rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.bounding_callback, queue_size=1)				# Bounding-Box-Array
		rospy.Subscriber("/camera/aligned_depth_to_color/image_raw_rotated", Image, self.depth_callback, queue_size=1)		# Depth-Image aligned to Color-Image
		rospy.Subscriber("/camera/aligned_depth_to_color/camera_info", CameraInfo, self.cameraInfo_callback, queue_size=1)	# Camera Calibration

		# Wait for Subscribers to initialize
		rospy.sleep(1)

	# Publish Pose of object
	def pub_object_pose(self, xyz, rxyzw):
		pubPose = Pose()
		pubPose.position.x = float(xyz.x) / 1000
		pubPose.position.y = float(xyz.y) / 1000
		pubPose.position.z = float(xyz.z) / 1000
		pubPose.orientation.x = rxyzw[0]
		pubPose.orientation.y = rxyzw[1]
		pubPose.orientation.z = rxyzw[2]
		pubPose.orientation.w = rxyzw[3]

		self.object_pos_pub.publish(pubPose)

	# Get camera-info and make it accesible in the class
	def cameraInfo_callback(self, data):
		self.camInfo = data

	# Get RGB-image and deep-copy it
	def image_callback(self, data):
		# Convert from ROS-Image to CV2::Mat and create a deep copy
		try:
			cv_rgb_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
			self.cv_rgb_image = cv_rgb_image.copy()
		except CvBridgeError as e:
			print(e)

	# Get bounding boxes make them accesible in the class
	def bounding_callback(self, data):
		self.currentBoundingBoxes = data

	# Get Depth-image and deep-copy it
	def depth_callback(self, data):
		try:
			# Convert from ROS-Image to CV2::Mat
			cv_depth_image = self.bridge.imgmsg_to_cv2(data,"32FC1")
			self.cv_depth_image = cv_depth_image.copy()
			# Convert from CV2::Mat to Array with depth-Values
			depth_array = np.array(self.cv_depth_image, dtype=np.float32)
			self.depth_array = depth_array.copy()
		except CvBridgeError as e:
			print(e)

	# Locate the object - wait before to make sure all topics are up-to-date
	def refresh_center_pos(self):
		rospy.rostime.wallsleep(0.1)
		state = self.locateObject()
		return state

	# Calculate coordinates of object relative to camera
	def locateObject(self):
		# Check for object in found objects (bounding boxes)
		for bounding_box in self.currentBoundingBoxes.bounding_boxes:
			if (bounding_box.Class == self.objectClass or bounding_box.Class == "bowl"): 			# TODO delete bowl
				# Calculate center of object in x, y, z
				self.objCenterPX.x = bounding_box.xmin + (bounding_box.xmax - bounding_box.xmin)/2
				self.objCenterPX.y = bounding_box.ymin + (bounding_box.ymax - bounding_box.ymin)/2
				self.objCenterPX.z = self.depth_array[self.objCenterPX.y][self.objCenterPX.x]

				# Draw circle on center of bounding box
				cv2.circle(self.cv_depth_image,(self.objCenterPX.x, self.objCenterPX.y), 10, (0,0,0), -1)

				# Approximate if there is no depth-data
				if self.objCenterPX.z == 0:
					self.objCenterPX.z = self.distance_to_object_approximator(bounding_box.xmax - bounding_box.xmin)
				
				# Calculate and publish coordinates in mm (Center of image is zero)
				self.objCenterM = self.calculate_center_coordinates(self.objCenterPX.x, self.objCenterPX.y, self.objCenterPX.z)
				self.pub_object_pose(self.objCenterM, [0, 0, 0, 1])
				#print "Center of box in mm (x, y, z): {0:3.0f}, {1:3.0f}, {2:3.0f}".format(x, y, z)
				
				# Calculate the radius of the object for when located over it (smaller radius is radius of cup)
				obj_radius1 = (bounding_box.xmax - bounding_box.xmin) / 2
				obj_radius2 = (bounding_box.ymax - bounding_box.ymin) / 2
				if obj_radius1 > obj_radius2:
					self.obj_radiusPX = obj_radius2
				else:
					self.obj_radiusPX = obj_radius1 

				return True

		# Return True if found, otherwise False
		return False

	# Approximate distance to object if no depth-value in array. Interpolation function based on real measurements
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

	# Remove the background of a depth image in a specified distance
	def remove_bg(self, threshold, blurVal):
		# Set all pixels to 255 which are farer away then "threshold" (depends on robot pre-position)
		cv2.threshold(self.cv_depth_image_local, threshold, 255, cv2.THRESH_BINARY, self.cv_depth_image_local)

		# Set all pixel to 255 which are gripper (x >= 450)
		for x in range(450, 480-1):
			for y in range(0, 640-1):
					self.cv_depth_image_local[x][y] = 255

		# Convert the image to uint8 to make it processible with other functions
		self.cv_depth_image_local = np.uint8(self.cv_depth_image_local)

		# Blur the image to lose small regions which could be detected wrong
		self.cv_depth_image_local = cv2.medianBlur(self.cv_depth_image_local, blurVal)

	# Display the position of the calculated grasping-point in the RGB-image
	def disp_graspPoint(self):
		# Display the images
		output = np.hstack((cv2.cvtColor(self.cv_depth_image_local, cv2.COLOR_GRAY2BGR), self.rgb_image_local))

		# Do it twice so the image is displayed correctly
		cv2.imshow("Grasp-Point", output)
		cv2.waitKey(1)
		rospy.sleep(0.1)
		cv2.imshow("Grasp-Point", output)
		cv2.waitKey(1)

		inp = raw_input("Grasp Point correct? y/n: ")[0]
		return inp

	# Find the end of the handle of the cup (eoh = end of handle)
	def find_eoh(self):
		# Calculate the area around the center of the cup, where should be searched for the handle
		sizeFactor = 1.8
		start_y = int(self.objCenterPX_local.x - sizeFactor*self.obj_radiusPX_local)
		start_x = int(self.objCenterPX_local.y - sizeFactor*self.obj_radiusPX_local)
		end_y = int(self.objCenterPX_local.x + sizeFactor*self.obj_radiusPX_local)
		end_x = int(self.objCenterPX_local.y + sizeFactor*self.obj_radiusPX_local)
		if start_y < 0:
			start_y = 0
		if start_x < 0:
			start_x = 0
		if end_y >= len(self.cv_depth_image_local[1]):
			end_y = len(self.cv_depth_image_local[1])-1
		if end_x >= len(self.cv_depth_image_local):
			end_x = len(self.cv_depth_image_local)-1

		# Calculate the distance of every pixel which is black to the center of the cup to get end of handle
		biggestDist = 0.0
		biggestDistPos_px = Point()
		for x in range(start_x, end_x):
			for y in range(start_y, end_y):
				if self.cv_depth_image_local[x][y] == 0:
					dist = math.sqrt((self.objCenterPX_local.x-y)**2 + (self.objCenterPX_local.y-x)**2)
					if dist > biggestDist:
						biggestDist = dist
						biggestDistPos_px.x = y
						biggestDistPos_px.y = x
		# Store Point of Interest (end of handle)
		poi = biggestDistPos_px
		poi.x = int(poi.x)
		poi.y = int(poi.y)

		# Draw circles and lines for area of interest
		cv2.circle(self.rgb_image_local ,(start_y, start_x),2,(0,255,0),3)
		cv2.circle(self.rgb_image_local ,(start_y, end_x),2,(0,255,0),3)
		cv2.circle(self.rgb_image_local ,(end_y, start_x),2,(0,255,0),3)
		cv2.circle(self.rgb_image_local ,(end_y, end_x),2,(0,255,0),3)
		cv2.line(self.rgb_image_local, (start_y, start_x), (end_y, start_x), (150,150,0))
		cv2.line(self.rgb_image_local, (start_y, start_x), (start_y, end_x), (150,150,0))
		cv2.line(self.rgb_image_local, (end_y, end_x), (start_y, end_x), (150,150,0))
		cv2.line(self.rgb_image_local, (end_y, end_x), (end_y, start_x), (150,150,0))

		# Draw circle for point of interest (outest point)
		cv2.circle(self.rgb_image_local ,(poi.x, poi.y),2,(0,0,255),3)

		return poi

	# Calculate and publish pose where to grasp the cup
	def find_cup_graspPoint(self, threshold):
		# Copy values so they don't change during calculation
		self.objCenterPX_local = Point()
		self.objCenterPX_local.x = int(self.objCenterPX.x)
		self.objCenterPX_local.y = int(self.objCenterPX.y)
		self.obj_radiusPX_local = int(self.obj_radiusPX)
		self.cv_depth_image_local = self.cv_depth_image.copy()
		self.rgb_image_local = self.cv_rgb_image.copy()

		# Remove background and blur image
		self.remove_bg(threshold, 5)

		# Get end of handle
		poi = self.find_eoh()

		# Calculate grasp-point and grasp-pose if a poi (end of handle) has been found
		graspPoint = Point()
		if poi.x != 0:
			# Vector from poi to objCenter
			v = Point()
			v.x = self.objCenterPX_local.x - poi.x
			v.y = self.objCenterPX_local.y - poi.y
			# Length of vector v
			lv = math.sqrt((self.objCenterPX_local.x - poi.x)**2 + (self.objCenterPX_local.y - poi.y)**2)
			# Resize vector to unit vector
			v.x = v.x / lv
			v.y = v.y / lv

			# Grasp-Point is in the middle between the poi and where v intersects with the radius
			graspPoint.x = int(poi.x + v.x * (lv-self.obj_radiusPX_local)/2)
			graspPoint.y = int(poi.y + v.y * (lv-self.obj_radiusPX_local)/2)
			graspPoint.z = self.depth_array[graspPoint.y, graspPoint.x]

			# Calculation of graspAngle of vecotr v to axis
			# Make a vector along the x-axis
			x_axis = Point()
			x_axis.x = 1
			x_axis.y = 0
			lx_axis = 1
			lv = 1	# Set length to 1 because v is already resized
			dotProduct = v.x*x_axis.x + v.y*x_axis.y
			graspAngle = math.acos(dotProduct/(lv*lx_axis))

			# Detect if angle is positive or negative
			if poi.y > self.objCenterPX_local.y:
				graspAngle = -graspAngle
			print "Angle is: " + str(graspAngle*180/pi)
			# Graspable between 0 and 180 degrees
			if graspAngle < 0:
				graspAngle = pi + graspAngle
				print "Corrected to " + str(graspAngle*180/pi)

			quats = tf.transformations.quaternion_from_euler(graspAngle+pi, -pi/2, pi/2, 'rzyx')

			# Draw line from poi to center of cup
			cv2.line(self.rgb_image_local, (self.objCenterPX_local.x, self.objCenterPX_local.y), (poi.x, poi.y), (150,150,0))
			# Draw circle for border of cup
			cv2.circle(self.rgb_image_local ,(self.objCenterPX_local.x, self.objCenterPX_local.y), self.obj_radiusPX_local, (0,255,0),2)
			# Draw circle for center of cup
			cv2.circle(self.rgb_image_local ,(self.objCenterPX_local.x, self.objCenterPX_local.y), 2, (0,0,255),3)
			# Draw grasp-point
			cv2.circle(self.rgb_image_local ,(graspPoint.x, graspPoint.y), 2, (0,150,150),3)

			# Display grasp point and ask for user-input
			inp = self.disp_graspPoint()

			# If there is no depth value at grasp-point
			if graspPoint.z == 0:
				print "Depth-value invalid. Taking value from table height."
				graspPoint.z = threshold - 50

			# If user input sait point is correct and point is valid
			if graspPoint.z != 0 and inp == 'y':
				graspPoint = self.calculate_center_coordinates(graspPoint.x, graspPoint.y, graspPoint.z)
				self.pub_object_pose(graspPoint, quats)
				cv2.destroyAllWindows()
				return True

		# If user input said point is incorrect or handle has not been found
		print "Trying again..."
		cv2.destroyAllWindows()
		return False

	# Calculate and publish pose where to grasp the bottle
	def find_bottle_graspPoint(self, threshold):
		self.cv_depth_image_local = self.cv_depth_image.copy()
		self.rgb_image_local = self.cv_rgb_image.copy()

		# Remove background and blur image
		self.remove_bg(threshold, 39)
		
		# Find circles
		circles = cv2.HoughCircles(self.cv_depth_image_local, cv2.HOUGH_GRADIENT, 2, 200, param1=50,param2=30,minRadius=0,maxRadius=70)

		# If at least one circle has been detected
		if type(circles) == np.ndarray:
			circles = np.uint16(np.around(circles))		# circles[0] = x, 1 = y, 2 = R

			# Find biggest circle
			maxR = 0
			actI = 0
			biggestRIdx = 0
			for i in circles[0,:]:
				if i[2] > maxR:
					maxR = i[2]
					biggestRIdx = actI
				actI = actI + 1

			# Calculate Grasp-Point
			graspPoint = Point()
			graspPoint.x = int(circles[0][biggestRIdx][0])
			graspPoint.y = int(circles[0][biggestRIdx][1])
			graspPoint.z = self.depth_array[graspPoint.y, graspPoint.x]

			# Draw biggest circle and its center into image
			cv2.circle(self.rgb_image_local, (graspPoint.x, graspPoint.y), maxR, (0,255,0), 2)
			cv2.circle(self.rgb_image_local, (graspPoint.x, graspPoint.y), 2, (0,0,255), 3)

			# Display grasp point and ask for user-input
			inp = self.disp_graspPoint()

			# If there is no depth value at grasp-point
			if graspPoint.z == 0:
				print "Depth-value invalid. Taking value from table height."
				graspPoint.z = threshold - 50 

			# If user input sait point is correct and point is valid				
			if graspPoint.z != 0 and inp == 'y':
				graspPoint = self.calculate_center_coordinates(graspPoint.x, graspPoint.y, graspPoint.z + 50)		# +50 so it does not grasp at the top of bottle
				quats = tf.transformations.quaternion_from_euler(pi/2, 0, pi, 'rzyx')
				self.pub_object_pose(graspPoint, quats)
				cv2.destroyAllWindows()
				return True
			print "Trying again..."
			cv2.destroyAllWindows()
			return False
		else:
			print "No bottle detected."
			return False

def main(args):
	# Initialize ros-node and Class
	rospy.init_node('objectLocator', anonymous=True, disable_signals=True)
	img_proc = img_processing("cup")

	print img_proc.find_bottle_graspPoint(481)

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print ("Shutting down ROS object locator")

	# Close all Image-Windows 
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
