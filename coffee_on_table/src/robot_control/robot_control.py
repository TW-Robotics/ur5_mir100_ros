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

	'''ur5.moveToSearchPose()
	print imgProc.inner_in_outer()
	while imgProc.inner_in_outer() == False:
		print "Searching"
		ur5.searchObject()
		print "Dist: " + str(ur5.distToObj)
	imgProc.inner_in_outer()
	while ur5.distToObj > 0.8:
		imgProc.inner_in_outer()
		print "Found Cup... Following"
		ur5.followObject()
	print "Driving to Cup"
	'''
	ur5.moveToObject()
	
	print "Analyse depth-image"
	while imgProc.find_handle() == False:
#	while True:
		print "search"
		rospy.rostime.wallsleep(0.5)
		imgProc.inner_in_outer()
		inp = raw_input("search handle? y/n: ")[0]
		imgProc.find_handle()
		#inp = raw_input("Move robot? y/n: ")[0]
	print "Found grapping Position"
	ur5.moveToGrappingPose()
	print "At grapping position"

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print ("Shutting down ROS Cupfinder")

if __name__ == '__main__':
	main(sys.argv)
