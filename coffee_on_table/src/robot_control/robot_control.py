import rospy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from std_msgs.msg import Header

import pyrealsense2 as rs
import numpy as np
import cv2

cam_info_pub = rospy.Publisher("/camera/aligned_depth_to_color/camera_info", CameraInfo, queue_size=10)
colorImg_pub = rospy.Publisher("/camera/color/image_raw", Image, queue_size=10)
depthImg_pub = rospy.Publisher("/camera/aligned_depth_to_color/image_raw", Image, queue_size=10)

width, height = 640, 480

# Create a pipeline
pipeline = rs.pipeline()

#Create a config and configure the pipeline to stream
#  different resolutions of color and depth streams
config = rs.config()
config.enable_stream(rs.stream.depth, width, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, width, 480, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)

def main(args):
	rospy.init_node("Camera_Python_Wrapper")
	h = Header()
	h.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work

	# Getting the depth sensor's depth scale (see rs-align example for explanation)
	depth_sensor = profile.get_device().first_depth_sensor()
	depth_scale = depth_sensor.get_depth_scale()

	# Create an align object
	# rs.align allows us to perform alignment of depth frames to others frames
	# The "align_to" is the stream type to which we plan to align depth frames.
	align_to = rs.stream.color
	align = rs.align(align_to)

	# Streaming loop
	# TODO Timen?!
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
			
			pub_msgs(h, color_image, depth_image)

			# Render images
			depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
			images = np.hstack((bg_removed, depth_colormap))
			cv2.namedWindow('Align Example', cv2.WINDOW_AUTOSIZE)
			cv2.imshow('Align Example', images)
			cv2.waitKey(1)
	finally:
		pipeline.stop()

def pub_msgs(h, color_img, depth_img):
	cam_info_pub.publish(make_camera_msg(h))
	colorImg_pub.publish(make_colorImg_msg(h, color_img))
	depthImg_pub.publish(make_depthImg_msg(h, depth_img))

def make_camera_msg(h):
	camera_info_msg = CameraInfo()
	fx, fy = 616.741455078125, 616.919677734375
	cx, cy = 324.817626953125, 238.0455780029297
	camera_info_msg.header = h
	camera_info_msg.header.frame_id = "camera_color_optical_frame"
	camera_info_msg.distortion_model ="plumb_bob"
	camera_info_msg.width = width
	camera_info_msg.height = height
	camera_info_msg.K = [fx, 0, cx,
						 0, fy, cy,
						 0, 0, 1]
	camera_info_msg.D = [0, 0, 0, 0]
	camera_info_msg.P = [fx, 0, cx, 0,
						 0, fy, cy, 0,
						 0, 0, 1, 0]
	return camera_info_msg
	
# BILD CONVERTIEREN MIT CV2_BRIDGE?
def make_colorImg_msg(h, color_image):
	img_msg = Image()
	img_msg.header = h
	img_msg.header.frame_id = "camera_color_optical_frame"
	img_msg.height = height
	img_msg.width = width
	img_msg.encoding = "rgb8"
	img_msg.is_bigendian = 0
	img_msg.step = 1920
	img_msg.data = color_image
	return img_msg

def make_depthImg_msg(h, depth_image):
	img_msg = Image()
	img_msg.header = h
	img_msg.header.frame_id = "camera_color_optical_frame"
	img_msg.height = height
	img_msg.width = width
	img_msg.encoding = "16UC1"
	img_msg.is_bigendian = 0
	img_msg.step = 1280
	img_msg.data = depth_image
	return img_msg

'''
import pyrealsense2 as rs
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