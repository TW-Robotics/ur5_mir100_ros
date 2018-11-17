import cv2
import sys
import numpy as np

def main:
	detector 	= cv2.SimpleBlobDetector_create()
	params		= cv2.SimpleBlobDetector_Params()

	# Filter by Area.
	params.filterByArea = True
	params.minArea = 20000
	params.maxArea = 40000

	params.filterByCircularity = False
	params.filterByConvexity = False
	params.filterByInertia = True
	params.minInertiaRatio = 0.8
	params.minDistBetweenBlobs = 200

	detector = cv2.SimpleBlobDetector_create(params)

	overlay = im.copy()

	keypoints = detector.detect(im)
	for k in keypoints:
		cv2.circle(overlay, (int(k.pt[0]), int(k.pt[1])), int(k.size/2), (0, 0, 255), -1)
		cv2.line(overlay, (int(k.pt[0])-20, int(k.pt[1])), (int(k.pt[0])+20, int(k.pt[1])), (0,0,0), 3)
		cv2.line(overlay, (int(k.pt[0]), int(k.pt[1])-20), (int(k.pt[0]), int(k.pt[1])+20), (0,0,0), 3)

	opacity = 0.5
	cv2.addWeighted(overlay, opacity, im, 1 - opacity, 0, im)

	# Uncomment to resize to fit output window if needed
	#im = cv2.resize(im, None,fx=0.5, fy=0.5, interpolation = cv2.INTER_CUBIC)
	cv2.imshow("Output", im)

	k = cv2.waitKey(1) & 0xff
	if k == 27:
		break

