## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#####################################################
##              Align Depth to Color               ##
#####################################################

# First import the library
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2

# Create a pipeline
pipeline = rs.pipeline()

#Create a config and configure the pipeline to stream
#  different resolutions of color and depth streams
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)

def main(args):
	# Getting the depth sensor's depth scale (see rs-align example for explanation)
	depth_sensor = profile.get_device().first_depth_sensor()
	depth_scale = depth_sensor.get_depth_scale()
	print("Depth Scale is: " , depth_scale)

	# We will be removing the background of objects more than
	#  clipping_distance_in_meters meters away
	clipping_distance_in_meters = 1 #1 meter
	clipping_distance = clipping_distance_in_meters / depth_scale

	# Create an align object
	# rs.align allows us to perform alignment of depth frames to others frames
	# The "align_to" is the stream type to which we plan to align depth frames.
	align_to = rs.stream.color
	align = rs.align(align_to)

	# Streaming loop
	try:
		while True:
			# Get frameset of color and depth
			frames = pipeline.wait_for_frames()
			# frames.get_depth_frame() is a 640x360 depth image
			
			# Align the depth frame to color frame
			aligned_frames = align.process(frames)
			
			# Get aligned frames
			aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
			color_frame = aligned_frames.get_color_frame()
			
			# Validate that both frames are valid
			if not aligned_depth_frame or not color_frame:
				continue
			
			depth_image = np.asanyarray(aligned_depth_frame.get_data())
			color_image = np.asanyarray(color_frame.get_data())
			
			# Remove background - Set pixels further than clipping_distance to grey
			grey_color = 153
			depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
			bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)
			
			# Render images
			depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
			images = np.hstack((bg_removed, depth_colormap))
			cv2.namedWindow('Align Example', cv2.WINDOW_AUTOSIZE)
			cv2.imshow('Align Example', images)
			cv2.waitKey(1)
	finally:
		pipeline.stop()

'''import pyrealsense2 as rs
import numpy as np
import cv2

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

def main(args):
	try:
		while True:

			# Wait for a coherent pair of frames: depth and color
			frames = pipeline.wait_for_frames()
			depth_frame = frames.get_depth_frame()
			color_frame = frames.get_color_frame()
			if not depth_frame or not color_frame:
				continue

			# Convert images to numpy arrays
			depth_image = np.asanyarray(depth_frame.get_data())
			color_image = np.asanyarray(color_frame.get_data())

			# Apply colormap on depth image (image must be converted to 8-bit per pixel first)
			depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

			# Stack both images horizontally
			images = np.hstack((color_image, depth_colormap))

			# Show images
			cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
			cv2.imshow('RealSense', images)
			cv2.waitKey(1)

	finally:

		# Stop streaming
		pipeline.stop()
'''

'''# First import the library
import pyrealsense2 as rs

def main(args):
	# Create a context object. This object owns the handles to all connected realsense devices
	pipeline = rs.pipeline()
	pipeline.start()

	while True:
		# Create a pipeline object. This object configures the streaming camera and owns it's handle
		frames = pipeline.wait_for_frames()
		depth = frames.get_depth_frame()
		if not depth: continue

		# Print a simple text-based representation of the image, by breaking it into 10x20 pixel regions and approximating the coverage of pixels within one meter
		coverage = [0]*64
		for y in xrange(480):
			for x in xrange(640):
				dist = depth.get_distance(x, y)
				if 0 < dist and dist < 1:
					coverage[x/10] += 1

			if y%20 is 19:
				line = ""
				for c in coverage:
					line += " .:nhBXWW"[c/25]
				coverage = [0]*64
				print(line)
'''

'''import rospy
import sys
from ur5_control import ur5_controller
from find_mug_on_table import findover

def main(args):
	# Initialize ros-node and Class
	rospy.init_node('robotControl', anonymous=True)

	ur5 = ur5_controller.ur5Controler()
	imgProc = findover.rossinator()

	ur5.addObject()

	##### Searching for the object
	# Move the UR5 to the search-pose
	ur5.moveToSearchPose()
	# As long as the searched object is not visible
	while imgProc.refresh_center_pos() == False:
		print "Searching"
		ur5.searchObject()
		print "Distance to object: " + str(ur5.distToObj)

	zDist = 350-39
	##### Follow the found object
	while True:
		print "Found Cup... Following"
		imgProc.refresh_center_pos()
		ur5.followObject()
		print "Distance to object: " + str(ur5.distToObj)
		imgProc.refresh_center_pos()
		if ur5.isGoalReachable(zDist):
			break
	print "Driving to Cup"
	
	##### Driving over the object
	# Get the actual center position
	rospy.sleep(1)
	imgProc.refresh_center_pos()
	# Move over the object
	ur5.moveOverObject(zDist)

	print "Correcting Position"
	imgProc.refresh_center_pos()
	rospy.sleep(1)
	print imgProc.obj_center_pos.x
	print imgProc.obj_center_pos.y
	ur5.correctPositionXY(imgProc.obj_center_pos.x, -imgProc.obj_center_pos.y)
	
	##### Locating the handle of the cup
	print "Analyse depth-image"
	while True:
		print "Searching for handle..."
		imgProc.refresh_center_pos()
		state = imgProc.find_handle(zDist-50)	# Camera is about 50mm in front of TCP - TODO: Change when TCP changes
		if state == True:
			break
	print "Found grabbing Position"
	rospy.rostime.wallsleep(0.5)	# needed to get actual position
	ur5.moveToGrabbingPose(imgProc.alpha)
	print "At grabbing position"
	return True

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print ("Shutting down ROS Robot Control")
'''
if __name__ == '__main__':
	main(sys.argv)





'''import socket
import time

HOST = "192.168.12.248"    	# The remote host
PORT = 30002              	# The same port as used by the server

def main(args):
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.connect((HOST, PORT))
	#s.send ("set_digital_out(2,True)" + "\n")
	#time.sleep (1)

	#s.send ("rq_reset(\"1\")" + "\n")
	#time.sleep (1)

	#s.send ("rq_activate_and_wait(\"1\")" + "\n")
	#time.sleep (1)

	var = 0
	var = s.send ("rq_current_pos()" + "\n")
	time.sleep(1)

	#s.send ("rq_close_and_wait(\"1\")" + "\n")
	#time.sleep (1)

	#s.send ("popup(\"Messages\", title=\"The Headline in the Blue box\", blocking=True)" + "\n")
	#time.sleep (1)

	#s.send ("set_digital_out(2,False)" + "\n")
	#time.sleep (1)

	print var
	data = s.recv(1024)
	#print str(data)

	print "Good bye!"
'''