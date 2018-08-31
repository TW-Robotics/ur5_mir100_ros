import rospy
import sys
from ur5_control import ur5_controller
from find_mug_on_table import findover

def main(args):
	# Initialize ros-node and Class
	rospy.init_node('robotControl', anonymous=True)

	ur5 = ur5_controller.ur5Controler()
	imgProc = findover.rossinator()
	#while True:
	#	print "Center of box in mm (x, y, z): {0:3.0f}, {1:3.0f}, {2:3.0f}".format(imgProc.obj_center.x*1000, imgProc.obj_center.y*1000, imgProc.obj_center.z*1000)

	ur5.moveToSearchPose()
	print imgProc.inner_in_outer()
	while imgProc.inner_in_outer() == False:
		print "Searching"
		ur5.searchObject()
		print "Dist: " + str(ur5.distToObj)
	imgProc.inner_in_outer()
	while ur5.distToObj > 0.7:
		imgProc.inner_in_outer()
		print "Found Cup... Following"
		ur5.followObject()
	print "Driving to Cup"
	
	zDist = 350
	rospy.rostime.wallsleep(0.5)
	imgProc.inner_in_outer()
	rospy.rostime.wallsleep(0.5)
	ur5.moveToObject(zDist)
	imgProc.inner_in_outer()
	#print "Correcting Position"
	#ur5.correctPositionXY(imgProc.obj_center_pos.x, imgProc.obj_center_pos.y)
	#ur5.move_xyz(-imgProc.obj_center_pos.x/1000, -imgProc.obj_center_pos.y/1000, 0)
	#zDist = zDist - 40
	
	imgProc.inner_in_outer()
	zDist = zDist-50
	rospy.rostime.wallsleep(0.5)
	print "Analyse depth-image"
	#while True:
	while True:
		print "search"
		rospy.rostime.wallsleep(0.5)
		imgProc.inner_in_outer()
		#inp = raw_input("search handle? y/n: ")[0]
		state = imgProc.find_handle(zDist)
		if state == True:
			break
		#inp = raw_input("Move robot? y/n: ")[0]
	print "Found grapping Position"
	rospy.rostime.wallsleep(0.5)	# needed to get actual position
	ur5.refresh()
	ur5.moveToGrappingPose(imgProc.alpha)
	print "At grapping position"

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print ("Shutting down ROS Cupfinder")

if __name__ == '__main__':
	main(sys.argv)
