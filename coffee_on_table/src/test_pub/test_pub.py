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

def main(args):
	pub = rospy.Publisher('cmd_vel', geometry_msgs.msg.Twist, queue_size=10)
	pub1 = rospy.Publisher('move_base_simple/goal', geometry_msgs.msg.PoseStamped, queue_size=10)
	pub2 = rospy.Publisher('move_base/goal', MoveBaseActionGoal, queue_size=10)

	rospy.init_node('talker', anonymous=True)

	toDo = 1

	if toDo == 1:
		rate = rospy.Rate(10) # 10hz
		secCounter = 0
		myMsg1 = geometry_msgs.msg.Twist()
		myMsg1.linear.x = -0.1 # [m/s] -> For 1 second means 1 m
		while not rospy.is_shutdown():
			pub.publish(myMsg1)
			secCounter = secCounter + 1
			if secCounter == 10:
				break
			rate.sleep()

	if toDo == 2:
		myMsg = geometry_msgs.msg.PoseStamped()
		#myMsg.header.stamp = rospy.Time.now()	# optional
		#myMsg.header.frame_id = '/map'	# optional
		myMsg.pose.position.x = 13.35
		myMsg.pose.position.y = 6.65
		myMsg.pose.orientation.w = 1
		pub1.publish(myMsg)

	if toDo == 3:
		myGoalMsg = MoveBaseActionGoal()
		#myGoalMsg.header.stamp = rospy.Time.now()	# optional
		myGoalMsg.header.frame_id = '/map' # Note: the frame_id must be map
		#myGoalMsg.goal.target_pose.header.stamp = rospy.Time.now()	# optional
		#myGoalMsg.goal.target_pose.header.frame_id = '/map'	# optional
		myGoalMsg.goal.target_pose.pose.position.x = 13.35
		myGoalMsg.goal.target_pose.pose.position.y = 6.65
		myGoalMsg.goal.target_pose.pose.position.z = 0.0 # z must be 0.0 (no height in the map)

		#myGoalMsg.goal_id.stamp = rospy.Time.now()	# optional
		#myGoalMsg.goal_id.id = '10'	# optional

		# Orientation of the robot is expressed in the yaw value of euler angles
		angle = radians(45) # angles are expressed in radians
		quat = quaternion_from_euler(0.0, 0.0, angle) # roll, pitch, yaw
		myGoalMsg.goal.target_pose.pose.orientation = Quaternion(*quat.tolist())

		pub2.publish(myGoalMsg)

if __name__ == '__main__':
	try:
		main('test')
	except rospy.ROSInterruptException:
		pass