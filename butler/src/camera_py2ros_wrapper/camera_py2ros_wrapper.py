import rospy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from std_msgs.msg import Header

import pyrealsense2 as rs
import numpy as np
import cv2

from cv_bridge import CvBridge, CvBridgeError

''' Connects to the camera and publish its depth-image, color-image and the camera-info'''

class py2rosWrapper():
	def __init__(self):
		# Init Variables
		self.bridge = CvBridge()
		self.width = 640
		self.height = 480
		self.camera_intrinsics = 0

		# Init Publisher
		self.cam_info_pub = rospy.Publisher("/camera/aligned_depth_to_color/camera_info", CameraInfo, queue_size=10)
		self.colorImg_pub = rospy.Publisher("/camera/color/image_raw", Image, queue_size=10)
		self.depthImg_pub = rospy.Publisher("/camera/aligned_depth_to_color/image_raw", Image, queue_size=10)

		# Get and publish frames in a loop
		self.getAndPublish()

	def getAndPublish(self):
		# Create a pipeline
		pipeline = rs.pipeline()

		#Create a config and configure the pipeline
		config = rs.config()
		config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, 30)
		config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, 30)

		# Start streaming
		profile = pipeline.start(config)

		# Get the intrinsic camera parameters
		color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
		self.camera_intrinsics = color_profile.get_intrinsics()

		# Create an align object - it should be aligned to the color frame
		align = rs.align(rs.stream.color)
		try:
			print "Camera Node successfully launched!"
			# Streaming loop
			while True:
				# Get frameset of color and depth
				frames = pipeline.wait_for_frames()
				
				# Align the depth frame to color frame
				aligned_frames = align.process(frames)
				
				# Get aligned frames
				aligned_depth_frame = aligned_frames.get_depth_frame()
				color_frame = aligned_frames.get_color_frame()
				
				# Validate that both frames are valid
				if not aligned_depth_frame or not color_frame:
					continue
					
				# Fetch color and depth frames
	   			depth = frames.get_depth_frame()
	   			color = frames.get_color_frame()
				
				# Convert frames to an array to work with ROS
				depth_image = np.asanyarray(aligned_depth_frame.get_data())
				color_image = np.asanyarray(color_frame.get_data())
				
				# Publish the messages (cam_info, color-image, depth-image)
				self.pub_msgs(color_image, depth_image)

				# Render images
				#depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
				#images = np.hstack((color_image, depth_colormap))
				#cv2.namedWindow('Align Example', cv2.WINDOW_AUTOSIZE)
				#cv2.imshow('Align Example', images)
				#cv2.waitKey(1)
		
		except KeyboardInterrupt:
			rospy.signal_shutdown("KeyboardInterrupt")
			raise

		finally:
			pipeline.stop()

	# Publish the messages (cam_info, color-image, depth-image)
	def pub_msgs(self, color_img, depth_img):
		# Convert color and depth-image to ROS-Messages
		color_img_msg = self.bridge.cv2_to_imgmsg(color_img, "bgr8")
		depth_img_msg = self.bridge.cv2_to_imgmsg(depth_img, "16UC1")

		# Publish images and cam-info
		self.colorImg_pub.publish(color_img_msg)
		self.depthImg_pub.publish(depth_img_msg)
		self.cam_info_pub.publish(self.make_camera_msg())

	# Create Camera-Info-Message
	def make_camera_msg(self):
		# Create header for camera messages
		h = Header()
		h.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work

		# Create Camera-Info-Message
		camera_info_msg = CameraInfo()
		fx, fy = self.camera_intrinsics.fx, self.camera_intrinsics.fy
		cx, cy = self.camera_intrinsics.ppx, self.camera_intrinsics.ppy
		camera_info_msg.header = h
		camera_info_msg.header.frame_id = "camera_color_optical_frame"
		camera_info_msg.distortion_model ="plumb_bob"
		camera_info_msg.width = self.width
		camera_info_msg.height = self.height
		camera_info_msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
		camera_info_msg.K = [fx, 0, cx,
							 0, fy, cy,
							 0, 0, 1]
		camera_info_msg.D = [0, 0, 0, 0, 0]
		camera_info_msg.P = [fx, 0, cx, 0,
							 0, fy, cy, 0,
							 0, 0, 1, 0]

		return camera_info_msg

	# UNUSED: Create Color-Image-Message
	def make_colorImg_msg(self, h, color_image):
		img_msg = Image()
		img_msg.header = h
		img_msg.header.frame_id = "camera_color_optical_frame"
		img_msg.height = self.height
		img_msg.width = self.width
		img_msg.encoding = "rgb8"
		img_msg.is_bigendian = 0
		img_msg.step = 1920
		img_msg.data = color_image
		return img_msg

	# UNUSED: Create Depth-Image-Message
	def make_depthImg_msg(self, h, depth_image):
		img_msg = Image()
		img_msg.header = h
		img_msg.header.frame_id = "camera_color_optical_frame"
		img_msg.height = self.height
		img_msg.width = self.width
		img_msg.encoding = "16UC1"
		img_msg.is_bigendian = 0
		img_msg.step = 1280
		img_msg.data = depth_image
		return img_msg

def main(args):
	rospy.init_node("Camera_Python_Wrapper", disable_signals=True)
	py2rosWrapper()

if __name__ == '__main__':
	main(sys.argv)
