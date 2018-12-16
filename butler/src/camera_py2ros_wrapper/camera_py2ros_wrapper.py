import rospy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from std_msgs.msg import Header

import pyrealsense2 as rs
import numpy as np
import cv2

from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

cam_info_pub = rospy.Publisher("/camera/aligned_depth_to_color/camera_info", CameraInfo, queue_size=10)
colorImg_pub = rospy.Publisher("/camera/color/image_raw", Image, queue_size=10)
depthImg_pub = rospy.Publisher("/camera/aligned_depth_to_color/image_raw", Image, queue_size=10)

width, height = 640, 480

# Create a pipeline
pipeline = rs.pipeline()

#Create a config and configure the pipeline to stream
#  different resolutions of color and depth streams
config = rs.config()
config.enable_stream(rs.stream.depth, width, height, rs.format.z16, 30)
config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)

# Declare pointcloud object, for calculating pointclouds and texture mappings
pc = rs.pointcloud()
# We want the points object to be persistent so we can display the last cloud when a frame drops
points = rs.points()

def main(args):
	rospy.init_node("Camera_Python_Wrapper")
	h = Header()
	h.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
	
	# Getting the depth sensor's depth scale (see rs-align example for explanation)
	depth_sensor = profile.get_device().first_depth_sensor()
	depth_scale = depth_sensor.get_depth_scale()
	print depth_scale

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
			#profile = pipeline.get_active_profile()
			color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
			color_intrinsics = color_profile.get_intrinsics()
			#print color_intrinsics
			# frames.get_depth_frame() is a 640x360 depth image
			
			# Align the depth frame to color frame
			aligned_frames = align.process(frames)
			
			# Get aligned frames
			aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
			color_frame = aligned_frames.get_color_frame()
			
			# Validate that both frames are valid
			if not aligned_depth_frame or not color_frame:
				continue
				
			# Fetch color and depth frames
   			depth = frames.get_depth_frame()
   			color = frames.get_color_frame()
			
			# Tell pointcloud object to map to this color frame
			pc.map_to(color)
		
		   	# Generate the pointcloud and texture mappings
			points = pc.calculate(depth)
			
			depth_image = np.asanyarray(aligned_depth_frame.get_data())
			color_image = np.asanyarray(color_frame.get_data())
			#print type(color_image)
			#print type(aligned_depth_frame.get_data())
			
			pub_msgs(h, color_image, depth_image)

			# Render images
			depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
			images = np.hstack((color_image, depth_colormap))
			#cv2.namedWindow('Align Example', cv2.WINDOW_AUTOSIZE)
			#cv2.imshow('Align Example', images)
			#cv2.waitKey(1)
	
	except KeyboardInterrupt:
		rospy.signal_shutdown("KeyboardInterrupt")
		raise

	finally:
		pipeline.stop()

def pub_msgs(h, color_img, depth_img):
	cam_info_pub.publish(make_camera_msg(h))

	color_img_msg = bridge.cv2_to_imgmsg(color_img, "bgr8")
	depth_img_msg = bridge.cv2_to_imgmsg(depth_img, "16UC1")
	#print imgTest

	#color_img_str = np.ndarray.tostring(color_img)
	#depth_img_str = np.ndarray.tostring(depth_img)
	#print np.shape(depth_img)
	#print len(depth_img_str)

	colorImg_pub.publish(color_img_msg)
	depthImg_pub.publish(depth_img_msg)

	#colorImg_pub.publish(make_colorImg_msg(h, str(color_img_str)))
	#depthImg_pub.publish(make_depthImg_msg(h, str(depth_img_str)))
	#print type((color_img_str))
	#print len(color_img_str)
	#print depth_img.size
	#print len(str(depth_img))

def make_camera_msg(h):
	camera_info_msg = CameraInfo()
	fx, fy = 616.741455078125, 616.919677734375
	cx, cy = 324.818, 238.046
	#cx, cy = 387.7503356933594, 241.0522918701172
	camera_info_msg.header = h
	camera_info_msg.header.frame_id = "camera_color_optical_frame"
	camera_info_msg.distortion_model ="plumb_bob"
	camera_info_msg.width = width
	camera_info_msg.height = height
	camera_info_msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
	camera_info_msg.K = [fx, 0, cx,
						 0, fy, cy,
						 0, 0, 1]
	camera_info_msg.D = [0, 0, 0, 0, 0]
	camera_info_msg.P = [fx, 0, cx, 0,
						 0, fy, cy, 0,
						 0, 0, 1, 0]

	'''camera_info_msg = CameraInfo()
	fx, fy = 387.75033569335945, 320.2814636230469
	cx, cy = 324.817626953125, 238.0455780029297
	camera_info_msg.header = h
	camera_info_msg.header.frame_id = "camera_depth_optical_frame"
	camera_info_msg.distortion_model ="plumb_bob"
	camera_info_msg.width = width
	camera_info_msg.height = height
	camera_info_msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
	camera_info_msg.K = [fx, 0, cx, 
						 0, fy, cy,
						 0, 0, 1]
	camera_info_msg.D = [0, 0, 0, 0, 0]
	camera_info_msg.P = [fx, 0, cx, 0,
						 0, fy, cy, 0,
						 0, 0, 1, 0]'''

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

if __name__ == '__main__':
	main(sys.argv)
