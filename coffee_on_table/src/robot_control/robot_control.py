import rospy
import sys
from ur5_control import ur5_controller
from find_mug_on_table import findover
from  gripper_control import gripper_control
from mir_control import mir_control
#import mir_control.mir_control as mir

def main(args):
	# Initialize ros-node and Class
	rospy.init_node('robotControl', anonymous=True)

	#ur5 = ur5_controller.ur5Controler()
	#imgProc = findover.rossinator()
	#mir = mir_control.mirControler()
	gripper = gripper_control.gripper()

	# Make sure the gripper is open
	gripper.open()
	rospy.sleep(5)
	gripper.close()
	print gripper.hasGripped()

	#mir.moveToGoal(13.35, 6.66, 0)
	#mir.moveToGoal(9.5, 4.95, -174)
	#rospy.sleep(10)
	#mir.moveToGoal(9.6, 9.1, -16)

	##### Searching for the object
	# Move the UR5 to the search-pose
	ur5.moveToSearchPose()

	# As long as the searched object is not visible
	while imgProc.refresh_center_pos() == False:
		print "Searching"
		ur5.searchObject()
		print "Distance to object: " + str(ur5.distToObj)

	##### Move the MiR to the Object
	# Calculate world position with tfs
	imgProc.refresh_center_pos()
	goalPose =  mir.objToOrigin_pose
	mir.moveToGoal(goalPose.position.x, goalPose.position.y, 0)	

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

	##### Close the gripper to grasp object
	#gripper.close()
	return True

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print ("Shutting down ROS Robot Control")

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