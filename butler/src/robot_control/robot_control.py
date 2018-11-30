import rospy
import sys
from ur5_control import ur5_controller
from img_processing import img_processing
from  gripper_control import gripper_control
from mir_control import mir_control
#import mir_control.mir_control as mir

def main(args):
	# Initialize ros-node and Class
	rospy.init_node('robotControl', anonymous=True)

	ur5 = ur5_controller.ur5Controler()
	imgProc = img_processing.img_processing()
	mir = mir_control.mirControler()
	gripper = gripper_control.gripper()

	'''print "Analyse depth-image"
	while True:
		print "Searching for handle..."
		imgProc.refresh_center_pos()
		state = imgProc.find_handle(250)	# Camera is about 50mm in front of TCP - TODO: Change when TCP changes
		if state == True:
			break
	print "Found grabbing Position"
	return 1'''

	##### Make sure the gripper is open
	print "Calibrating gripper..."
	gripper.open()

	##### Move the MiR to the Search-Goal
	#print "Moving MiR to goal-position..."
	#mir.moveToGoal(13.35, 6.66, 0)
	#while mir.isAtGoal() == False:
#		rospy.sleep(1)
	#print "MiR is at goal-position"

	##### Searching for the object
	# Move the UR5 to the search-pose
	ur5.moveToSearchPose()

	# As long as the searched object is not visible
	i = 0
	while imgProc.refresh_center_pos() == False:
		print "Searching for object..."
		ur5.searchObject(i)
		i = i + 1
		# TODO GOES INTO LOOP
		print "Distance to object: " + str(ur5.distToObj)

	##### Move the MiR to the Object
	# Calculate world position with tfs
	imgProc.refresh_center_pos()
	rospy.rostime.wallsleep(0.5)
	ur5.refresh()
	#goalPose =  mir.objToOrigin_pose
	#mir.moveToGoal(goalPose.position.x, goalPose.position.y, 0)	

	zDist = 250 - 41
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
	ur5.correctPositionXY(imgProc.obj_center_pos.y, imgProc.obj_center_pos.x)
	
	##### Locating the handle of the cup
	print "Analyse depth-image"
	while True:
		print "Searching for handle..."
		imgProc.refresh_center_pos()
		state = imgProc.find_handle(300)	# Camera is about 50mm in front of TCP - TODO: Change when TCP changes
		if state == True:
			break
	print "Found grabbing Position"
	rospy.rostime.wallsleep(0.5)	# needed to get actual position
	ur5.moveToGrabbingPose(imgProc.alpha)
	print "At grabbing position"

	##### Close the gripper to grasp object
	gripper.close()
	rospy.sleep(5)
	if gripper.hasGripped() == True:
		print "Successfully grasped object!"
	else:
		print "Error grasping object!"
		return False

	return True

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print ("Shutting down ROS Robot Control")

if __name__ == '__main__':
	main(sys.argv)
