import rospy
import sys
from ur5_control import ur5_controller
from img_processing import obj_localization
from gripper_control import gripper_control
from mir_control import mir_control
from mission_control import mission_control

def main(args):
	try:
		# Initialize ros-node and Class
		rospy.init_node('robotControl', anonymous=True, disable_signals=True)

		# Check user-inputs and store them in variables
		if len(args) < 4:
			print "  ERROR: Too few arguments given."
			print "  PARAMETERS: pickUp goal name"
			print "              putDown goal name"
			print "              objectToSearch ['cup' or 'bottle']"
			return -1
		else:
			pickUp = mission_control.goal(args[1])
			putDown = mission_control.goal(args[2])
			objectToSearch = args[3]
			if pickUp.name == "invalid" or putDown.name == "invalid":
				print "  ERROR: Invalid goal name! Check names of goals with file mission_control.py."
				return
			if objectToSearch != "cup" and objectToSearch != "bottle":
				print "  ERROR: Object-to-Search must be either 'cup' or 'bottle'."
				return
		print "Searching for ", objectToSearch, " at "
		pickUp.display()
		print "transporting it to "
		putDown.display()

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

		##### Move the UR to the Driving-Pose
		ur5.moveToDrivingPose()

		##### Move the MiR to the Search-Goal
		inp = raw_input("Move MiR robot? y/n: ")[0]
		if (inp == 'y'):
			print "Moving MiR to goal-position..."
			mir.moveToGoal(pickUp.posx, pickUp.posy, pickUp.rz)
			while mir.isAtGoal(0.2, 0.1) == False:
				rospy.sleep(1)
		print "MiR is at goal-position"

		##### Searching for the object
		# Move the UR5 to the search-pose
		print "Moving robot to search position..."
		ur5.moveToSearchPose(pickUp.orientation, pickUp.height)

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
			rospy.rostime.wallsleep(0.5)
			imgProc.refresh_center_pos()
			rospy.rostime.wallsleep(0.5)
			if ur5.isGoalReachable(zDist):
				break
			else:
				print "Goal is not reachable. Please move object!"

		##### Driving over the object
		print "Goal is reachable. Driving over cup..."
		ur5.moveOverObject(zDist, pickUp.height)

		if objectToSearch == "cup":
			print "Correcting Position..."
			imgProc.refresh_center_pos()
			ur5.move_xyz(float(imgProc.objCenterM.y)/1000, float(imgProc.objCenterM.x)/1000, 0)

		##### Locating the grasping point
		print "Analysing depth-image..."
		while True:
			print "Searching for grasping point..."
			imgProc.refresh_center_pos()
			threshold = ur5.tcp_to_floor() + 81 - pickUp.height 		# + 81 weil Kamera nicht am TCP ist
			print threshold
			if objectToSearch == "cup":
				state = imgProc.find_cup_graspPoint(threshold)		    # TODO Test code
			elif objectToSearch == "bottle":
				state = imgProc.find_bottle_graspPoint(threshold)		# WAR zDist + 81 (=381)
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

		##### Move the robot up
		ur5.move_xyz(0, 0, 0.1)
		ur5.moveToTransportPose(objectToSearch)

		##### Move the MiR to the Goal
		inp = raw_input("Move MiR robot? y/n: ")[0]
		if (inp == 'y'):
			print "Moving MiR to goal-position..."
			mir.moveToGoal(putDown.posx, putDown.posy, putDown.rz)
			while mir.isAtGoal(0.2, 0.1) == False:
				rospy.sleep(1)
		print "MiR is at goal-position"

		##### Move the UR to putDownPose and open gripper
		ur5.layDown(objectToSearch, putDown.orientation, pickUp.height, putDown.height)
		gripper.open()
		rospy.sleep(5)

		##### Move the UR back to safe pose
		ur5.move_xyz(0, 0, 0.1)
		ur5.moveToDrivingPose()
		return True
	
	except KeyboardInterrupt:
		print ("Shutting down ROS Robot Control")
		return False

if __name__ == '__main__':
	main(sys.argv)
