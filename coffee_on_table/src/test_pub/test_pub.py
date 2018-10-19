#!/usr/bin/env python
# license removed for brevity
import rospy
import std_msgs.msg
from std_msgs.msg import String
import geometry_msgs.msg

def main(args):
	pub = rospy.Publisher('cmd_vel', geometry_msgs.msg.Twist, queue_size=10)
	pub1 = rospy.Publisher('move_base_node/current_goal', geometry_msgs.msg.PoseStamped, queue_size=10)

	rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	#while not rospy.is_shutdown():
	#myMsg = geometry_msgs.msg.Twist()
	#myMsg.linear.x = -0.5
	#pub.publish(myMsg)
	myMsg = geometry_msgs.msg.PoseStamped()
	h = std_msgs.msg.Header()
	h.stamp = rospy.Time.now()
	#myMsg.header = h
	myMsg.pose.position.x = 13.35
	myMsg.pose.position.y = 6.65
	myMsg.pose.orientation.w = 1
	pub1.publish(myMsg)
	rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass