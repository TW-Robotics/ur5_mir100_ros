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

#cv_bridge
from cv_bridge import CvBridge, CvBridgeError

# Ros Messages
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBoxes

#instantiate
bridge = CvBridge()
VERBOSE=False
class rossinator:
	mug_pub = rospy.Publisher("/roi",CompressedImage, queue_size=1)
	currentBoundingBoxes = BoundingBoxes()
	#encoded_img

	def __init__(self):
		rospy.Subscriber("/darknet_ros/detection_image", Image, self.image_callback, queue_size=1)
		rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.bounding_callback, queue_size=1)

	def image_callback(self, data):
		img_np_arr = np.fromstring(data.data, np.uint8)
		encoded_img = cv2.imdecode(img_np_arr, cv2.IMREAD_COLOR)
		if self.coffee_on_table(encoded_img):
			print('I want myself some of that coffee sir')	    
		#save image to some local


	def bounding_callback(self, data):
		self.currentBoundingBoxes = data		


	def coffee_on_table(self, encoded_img):
		#idx_table = self.find_bounding_box("person", 0)

		for outer_box in self.currentBoundingBoxes.bounding_boxes:
			if outer_box.Class == 'person':
				for inner_box in self.currentBoundingBoxes.bounding_boxes:
					if inner_box.Class == 'cup' and inner_box.xmin > outer_box.xmin and inner_box.ymin > outer_box.ymin and inner_box.xmax < outer_box.xmax and inner_box.ymax < outer_box.ymax:
						print('we made it')

		#I return 1, if there is ore or more mugs on a table in sight
		#and 0 otherwise

	def find_bounding_box(self, class_name, flag):
		probability = 0
		size = 0
		idx = -1
		for index, box in enumerate(self.currentBoundingBoxes.bounding_boxes):
			if box.Class == class_name and box.probability > probability:
				[idx, probability] = [index, box.probability]
				print(probability)
				print(box.probability)
		return idx
		#returns index in box array with highest 
			#0 -- probability	
			#1 -- size	

def main(args):
	'''Initializes and cleanup ros node'''
	rospy.init_node('image_listener', anonymous=True)
	haengst = rossinator()

	try:
	    rospy.spin()
	except KeyboardInterrupt:
		print ("Shutting down ROS Image feature detector module")

	cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)

