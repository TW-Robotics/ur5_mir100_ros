import rospy
import sys
from ur5_control import ur5_controller
from find_mug_on_table import findover

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

def main(args):
	# Initialize ros-node and Class
	rospy.init_node('robotControl', anonymous=True)

	ur5 = ur5_controller.ur5Controler()
	imgProc = findover.rossinator()

	##### Searching for the object
	# Move the UR5 to the search-pose
	ur5.moveToSearchPose()
	# As long as the searched object is not visible
	while imgProc.refresh_center_pos() == False:
		print "Searching"
		ur5.searchObject()
		print "Distance to object: " + str(ur5.distToObj)

	##### Follow the found object
	while ur5.distToObj > 0.7:
		print "Found Cup... Following"
		imgProc.refresh_center_pos()
		ur5.followObject()
		print "Distance to object: " + str(ur5.distToObj)
	print "Driving to Cup"
	
	##### Driving over the object
	zDist = 350
	# Get the actual center position
	imgProc.refresh_center_pos()
	# Move over the object
	ur5.moveOverObject(zDist)

	#print "Correcting Position"
	#ur5.correctPositionXY(imgProc.obj_center_pos.x, imgProc.obj_center_pos.y)
	#ur5.move_xyz(-imgProc.obj_center_pos.x/1000, -imgProc.obj_center_pos.y/1000, 0)
	#zDist = zDist - 40
	
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

if __name__ == '__main__':
	main(sys.argv)
