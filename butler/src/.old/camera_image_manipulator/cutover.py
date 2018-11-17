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

#bridge
from cv_bridge import CvBridge, CvBridgeError

# Ros Messages
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBoxes

#instantiate
bridge = CvBridge()
VERBOSE=False
currentBoundingBoxes = BoundingBoxes()
look_mum_i_wrote_a_publisher = rospy.Publisher("/roi",CompressedImage, queue_size=1)


def callback(data):
    img_np_arr = np.fromstring(data.data, np.uint8)

    encoded_img = cv2.imdecode(img_np_arr, cv2.IMREAD_COLOR)
    
    for box in currentBoundingBoxes.bounding_boxes:
        if box.Class == "person" and box.probability >0.9:
            firstBox = currentBoundingBoxes.bounding_boxes[0]

            cropped_encoded_img = encoded_img[firstBox.ymin:firstBox.ymax,firstBox.xmin:firstBox.xmax]
            cv2.imshow("encoded_image",cropped_encoded_img)
            cv2.waitKey(1)
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg', encoded_img)[1]).tostring()
            look_mum_i_wrote_a_publisher.publish(msg)
            return

def bounding_callback(data):
    global currentBoundingBoxes
    currentBoundingBoxes = data
    for box in currentBoundingBoxes.bounding_boxes:
        if box.Class == "person":
            print("I See YOU!!")
            return
    print("Where you at? :(")
        
def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('image_listener', anonymous=True)
    rospy.Subscriber("/webcam/image_raw/compressed", CompressedImage, callback, queue_size=1)
    rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, bounding_callback, queue_size=1)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()




if __name__ == '__main__':
    main(sys.argv)

