import rospy
import sys
from ur5_control import ur5_controller
from img_processing import obj_localization
from gripper_control import gripper_control
from mir_control import mir_control

def main(args):
	try:
		# Initialize ros-node and Class
		rospy.init_node('robotControl', anonymous=True, disable_signals=True)

		# Check user-inputs and store them in variables
		if len(args) < 7:
			print "  ERROR: Too few arguments given."
			print "  PARAMETERS: MiR-GoalX [m]"
			print "              MiR-GoalY [m]"
			print "              MiR-Orientation [degrees]"
			print "              objectToSearch ['cup' or 'bottle']"
			print "              UR-SearchDirection ['left' or 'right']"
			print "              tableHeight [mm]"
			return -1
		else:
			mirGoalX = args[1]
			mirGoalY = args[2]
			mirOrientation = args[3]
			objectToSearch = args[4]
			searchDirection = args[5]
			tableHeight = args[6]
			if searchDirection != "left" and searchDirection != "right":
				print "  ERROR: Search-Direction must be either 'left' or 'right'."
			if objectToSearch != "cup" and objectToSearch != "bottle":
				print "  ERROR: Object-to-Search must be either 'cup' or 'bottle'."				

		ur5 = ur5_controller.ur5Controler()
		imgProc = obj_localization.img_processing(objectToSearch)
		mir = mir_control.mirControler()
		gripper = gripper_control.gripper()

		ur5.checkBeforeDo = True
		ur5.speed = 0.1

		##### Make sure the gripper is open
		#print "Calibrating gripper..."
		#gripper.open()
		#rospy.sleep(5)

		##### Move the MiR to the Search-Goal
		#print "Moving MiR to goal-position..."
		#mir.moveToGoal(mirGoalX, mirGoalY, mirOrientation)
		#while mir.isAtGoal(0.2, 0.1) == False:
		#	rospy.sleep(1)
		#print "MiR is at goal-position"

		##### Searching for the object
		# Move the UR5 to the search-pose
		print "Moving robot to search position..."
		ur5.moveToSearchPose(searchDirection)

		# As long as the searched object is not visible
		posID = 0
		while imgProc.refresh_center_pos() == False:
			print "Searching for object..."
			if ur5.searchObject(posID) == False:
				print "No object found!"
				return False
			posID = posID + 1
			rospy.rostime.wallsleep(1)

		zDist = 300		# TODO make variable with table height?!
		##### Follow the found object - center it
		while True:
			print "Found Object... Centering"
			imgProc.refresh_center_pos()
			ur5.followObject()
			#print "Distance to object: " + str(ur5.distToObj)
			rospy.rostime.wallsleep(0.5)
			imgProc.refresh_center_pos()
			rospy.rostime.wallsleep(0.5)
			if ur5.isGoalReachable(zDist):
				break
			else:
				print "Goal is not reachable. Please move object!"

		##### Driving over the object
		print "Goal is reachable. Driving over cup..."
		ur5.moveOverObject(zDist)

		if objectToSearch == "cup":
			print "Correcting Position..."
			imgProc.refresh_center_pos()
			ur5.move_xyz(float(imgProc.objCenterM.y)/1000, float(imgProc.objCenterM.x)/1000, 0)

		#zDist = 300
		##### Locating the grasping point
		print "Analysing depth-image..."
		while True:
			print "Searching for grasping point..."			# TODO add table-height to grasp-detection to not hurt UR
			imgProc.refresh_center_pos()
			if objectToSearch == "cup":
				state = imgProc.find_handle(zDist + 81)		# + 81 weil Kamera nicht am TCP ist
			elif objectToSearch == "bottle":
				state = imgProc.find_bottle(zDist + 81)
			if state == True:
				break
		print "Found grasping point."
		print "Moving to grasping point..."
		rospy.rostime.wallsleep(0.5)	# needed to get actual position
		if objectToSearch == "bottle":
			ur5.moveToPreGrabbingPoseBottle()
		ur5.moveToGrabbingPose()
		print "At grabbing position. Closing gripper."

		##### Close the gripper to grasp object
		gripper.close()
		rospy.sleep(5)
		if gripper.hasGripped() == True:
			print "Successfully grasped object!"
		else:
			print "Error grasping object!"
			return False

		##### Move the robot up and down again
		ur5.move_xyz(0, 0, 0.1)
		ur5.move_xyz(0, 0, -0.1)

		gripper.open()
		rospy.sleep(5)

		ur5.move_xyz(0, 0, 0.1)
		return True
	
	except KeyboardInterrupt:
		print ("Shutting down ROS Robot Control")
		return False

if __name__ == '__main__':
	main(sys.argv)
