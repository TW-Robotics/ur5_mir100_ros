#!/usr/bin/env python
# license removed for brevity
import rospy
import std_msgs.msg
from std_msgs.msg import String
import geometry_msgs.msg
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal
from math import radians, degrees
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from ur_msgs.msg import IOStates

ioStates = IOStates()
trueVar = True

'''pub4 = rospy.Publisher('/ur_driver/io_states', IOStates, queue_size=10)

def iostatesCallback(data):
	global ioStates
	ioStates = data
	print "BEFORE"
	print ioStates.digital_out_states[4].state
	ioStates.digital_out_states[4].state = trueVar
	print "AFTER"
	print ioStates.digital_out_states[4].state
	pub4.publish(ioStates)'''

def main(args):
	pub = rospy.Publisher('cmd_vel', geometry_msgs.msg.Twist, queue_size=10)
	pub1 = rospy.Publisher('move_base_simple/goal', geometry_msgs.msg.PoseStamped, queue_size=10)
	pub2 = rospy.Publisher('move_base/goal', MoveBaseActionGoal, queue_size=10)
	pub3 = rospy.Publisher('/ur_driver/URScript', String, queue_size=10)
	sub = rospy.Subscriber('/ur_driver/io_states', IOStates, iostatesCallback)

	rospy.init_node('talker', anonymous=True)
	#rospy.spin()
	#return

	toDo = 5

	#if toDo == 5:
		#global ioStates
		#ioStates.digital_out_states[4].state = trueVar
		#print "AFTER"
		#print ioStates.digital_out_states[4].state
		#pub4.publish(ioStates)

	if toDo == 4:
		pub3.publish("set_standard_digital_out(2,True)")

	if toDo == 1:
		rate = rospy.Rate(10) # 10hz
		secCounter = 0
		myMsg1 = geometry_msgs.msg.Twist()
		myMsg1.linear.x = -0.1 # [m/s] -> For 1 second means 1 m
		while not rospy.is_shutdown():
			pub.publish(myMsg1)
			secCounter = secCounter + 1
			if secCounter == 20:	# 30 = ca. 30 - 35 cm
				break
			rate.sleep()

	if toDo == 2:
		myMsg = geometry_msgs.msg.PoseStamped()
		#myMsg.header.stamp = rospy.Time.now()	# optional
		myMsg.header.frame_id = '/map'
		myMsg.pose.position.x = 14.35
		myMsg.pose.position.y = 6.65
		myMsg.pose.orientation.w = 1
		pub1.publish(myMsg)

	if toDo == 3:
		myGoalMsg = MoveBaseActionGoal()
		#myGoalMsg.header.stamp = rospy.Time.now()	# optional
		myGoalMsg.header.frame_id = '/map' # Note: the frame_id must be map
		#myGoalMsg.goal.target_pose.header.stamp = rospy.Time.now()	# optional
		myGoalMsg.goal.target_pose.header.frame_id = '/map'	# optional
		myGoalMsg.goal.target_pose.pose.position.x = 13.35
		myGoalMsg.goal.target_pose.pose.position.y = 6.65
		myGoalMsg.goal.target_pose.pose.position.z = 0.0 # z must be 0.0 (no height in the map)

		myGoalMsg.goal.target_pose.pose.orientation.w = 1
		#myGoalMsg.goal_id.stamp = rospy.Time.now()	# optional
		#myGoalMsg.goal_id.id = '10'	# optional

		# Orientation of the robot is expressed in the yaw value of euler angles
		#angle = radians(45) # angles are expressed in radians
		#quat = quaternion_from_euler(0.0, 0.0, angle) # roll, pitch, yaw
		#myGoalMsg.goal.target_pose.pose.orientation = Quaternion(*quat.tolist())

		#myGoalMsg.goal.target_pose.pose.position.x = 9.15
		#myGoalMsg.goal.target_pose.pose.position.y = 5.7
		#myGoalMsg.goal.target_pose.pose.orientation.z = 1
		#myGoalMsg.goal.target_pose.pose.orientation.w = 0

		pub2.publish(myGoalMsg)

if __name__ == '__main__':
	try:
		main('test')
	except rospy.ROSInterruptException:
		pass