import rosbridge
import cPickle
import time
import math
import json
import rospy
import std_msgs.msg

# values are saved in a list and saved globally for easy access
data = {
	'pose' : [],
	'velocity' : [],
	'odometry' : [],
	'amcl' : [],
	'imu' : [],
	'encoders' : [],
	'goal' : []
}

robot = None

# default port
port = 9090
sim_url = "10.211.55.4"

# run this first
def boot():
	print "create new robot (1)\nimport old robot(0)"
	robot_noload = input()

	if robot_noload == 1:
		print "Enter new robot url: "
		robot_url = input()
		print "robot url ", robot_url
		print "this will be saved as your new robot"

		with open('previous_ip.p','wb') as previous:
			cPickle.dump(robot_url, previous)

		robot = rosbridge.RosbridgeSetup(robot_url, port)

	elif robot_noload == 0:
		with open('previous_ip.p', 'rb') as previous:
			robot_url = cPickle.load(previous)
		print "robot url", robot_url
		robot = rosbridge.RosbridgeSetup(robot_url, port)
	else:
		raise ValueError("Input value 1 and 0 are valid, you entered: %d" % robot_load)

	# subscriped services i.e. services we are interested in hearing from
	robot.subscribe(topic='/robot_pose',callback=callback_robot_pose)
	robot.subscribe(topic='/cmd_vel',callback=callback_cmd_vel)
	robot.subscribe(topic='/odom_comb', callback=callback_odometry)
	robot.subscribe(topic='/amcl_pose', callback=callback_amcl_pose)
	robot.subscribe(topic='/MC/encoders', callback=callback_encoders)
	robot.subscribe(topic='/imu_data', callback=callback_imu_data)
	robot.subscribe(topic='/move_base_node/current_goal', callback=callback_goal)
	return robot

# --- move ---
#geometry_msgs/Vector3 linear
#  float64 x
#  float64 y
#  float64 z
#geometry_msgs/Vector3 angular
#  float64 x
#  float64 y
#  float64 z

# only for development
def boot_sim():
	print "Simulation(1), non-simulation(0)"
	sim = input()

	# insert robot_url
	if sim == 1:
		robot_url = sim_url
		print "robot url", robot_url
		robot = rosbridge.RosbridgeSetup(robot_url, port)

	elif sim == 0:
		print "new robot(1), import old robot(0)"
		robot_load = input()

		if robot_load == 1:
			robot_url = input()
			print "robot url", robot_url
			print "this will be saved as your new robot"

			with open('previous_ip.p','wb') as previous:
				cPickle.dump(robot_url, previous)

			robot = rosbridge.RosbridgeSetup(robot_url, port)

		elif robot_load == 0:
			with open('previous_ip.p', 'rb') as previous:
				robot_url = cPickle.load(previous)
			print "robot url", robot_url
			robot = rosbridge.RosbridgeSetup(robot_url, port)
		else:
			raise ValueError("Input value 1 and 0 are valid, you entered %d" % robot_load)

	else:
		raise ValueError("Input value 1 and 0 are valid, you entered: %d" % sim)
	# subscriped services i.e. services we are interested in hearing from
	robot.subscribe(topic='/robot_pose',callback=callback_robot_pose)
	robot.subscribe(topic='/cmd_vel',callback=callback_cmd_vel)
	robot.subscribe(topic='/odom_comb', callback=callback_odometry)
	robot.subscribe(topic='/amcl_pose', callback=callback_amcl_pose)
	robot.subscribe(topic='/MC/encoders', callback=callback_encoders)
	robot.subscribe(topic='/imu_data', callback=callback_imu_data)
	return robot

# movement of robot is controlled through linear and angular vector
def move(self, linear, angular):
	try:
		self.publish(
			"/cmd_vel",{
				"linear" : {
					"x" : linear[0],
					"y" : linear[1],
					"z" : linear[2],
					},
				"angular" : {
					"x" : angular[0],
					"y" : angular[1],
					"z" : angular[2]
					}
				})
	except:
		raise ValueError("Lost connection to MiR in move()")

def moveToGoal(self):
	rospy.init_node('myPub', anonymous=True)
	try:
		h = std_msgs.msg.Header()
		h.stamp=rospy.Time.now()
		#print h

		self.publish(
			"/move_base_node/current_goal",{
				"header": {
					"seq": 4,
					"stamp": {
						"secs" : h.stamp.secs,
						"nsecs" : h.stamp.nsecs+8041,
						},
					"frame_id": h.frame_id
					},
				"pose" : {
					"position" : {
						"x" : 6.94,
						"y" : 9.134,
						"z" : 0,
						},
					"orientation" : {
						"x" : 0,
						"y" : 0,
						"z" : 0.945288219951,
						"w" : 0.326236388563
						}
					}
				})
		'''self.publish(
			"/move_base_node/current_goal",{
				"pose" : {
					"position" : {
						"x" : 6.94,
						"y" : 9.134,
						"z" : 0,
						},
					"orientation" : {
						"x" : 0,
						"y" : 0,
						"z" : 0.945288219951,
						"w" : 0.326236388563
						}
					}
				})'''
	except:
		raise ValueError("Lost connection to MiR in moveToGoal()")

def shutdown(self):
	try:
		robot.unsubscribe(topic='/robot_pose')
		robot.unsubscribe(topic='/cmd_vel')
		robot.unsubscribe(topic='/odom_comb')
		robot.unsubscribe(topic='/amcl_pose')
		robot.unsubscribe(topic='/MC/encoders')
		robot.unsubscribe(topic='/imu_data')
	except:
		raise ValueError("Lost connection to MiR in shutdown")


def callback_goal(msg):
	data['goal'] = msg

# ---ROBOT POSE ---
#geometry_msgs/Point position
#  float64 x
#  float64 y
#  float64 z
#geometry_msgs/Quaternion orientation
#  float64 x
#  float64 y
#  float64 z
#  float64 w
def callback_robot_pose(msg):
	data['pose'] = msg

# ---COMMAND VELOCITY---
#geometry_msgs/Vector3 linear
#  float64 x
#  float64 y
#  float64 z
#geometry_msgs/Vector3 angular
#  float64 x
#  float64 y
#  float64 z
def callback_cmd_vel(msg):
	data['velocity'] = msg

# ---ODOMETRY--- not available in simulation
#std_msgs/Header header
#  uint32 seq
#  time stamp
#  string frame_id
#string child_frame_id
#geometry_msgs/PoseWithCovariance pose
#  geometry_msgs/Pose pose
#    geometry_msgs/Point position
#      float64 x
#      float64 y
#      float64 z
#    geometry_msgs/Quaternion orientation
#      float64 x
#      float64 y
#      float64 z
#      float64 w
#  float64[36] covariance
#geometry_msgs/TwistWithCovariance twist
#  geometry_msgs/Twist twist
#    geometry_msgs/Vector3 linear
#      float64 x
#      float64 y
#      float64 z
#    geometry_msgs/Vector3 angular
#      float64 x
#      float64 y
#      float64 z
#  float64[36] covariance
def callback_odometry(msg):
	data['odometry'] = msg

#---AMCL POSE---
#std_msgs/Header header
#  uint32 seq
#  time stamp
#  string frame_id
#geometry_msgs/PoseWithCovariance pose
#  geometry_msgs/Pose pose
#    geometry_msgs/Point position
#      float64 x
#      float64 y
#      float64 z
#    geometry_msgs/Quaternion orientation
#      float64 x
#      float64 y
#      float64 z
#      float64 w
#  float64[36] covariance 
def callback_amcl_pose(msg):
	data['amcl'] = msg

# ---ENCODERS---
#std_msgs/Header header
#  uint32 seq
#  time stamp
#  string frame_id
#sdc21x0/Encoders encoders
#  float32 time_delta
#  int32 left_wheel
#  int32 right_wheel
def callback_encoders(msg):
	data['encoders'] = msg

# ---IMU--- - not available in simulation
#std_msgs/Header header
#  uint32 seq
#  time stamp
#  string frame_id
#geometry_msgs/Quaternion orientation
#  float64 x
#  float64 y
#  float64 z
#  float64 w
#float64[9] orientation_covariance
#geometry_msgs/Vector3 angular_velocity
#  float64 x
#  float64 y
#  float64 z
#float64[9] angular_velocity_covariance
#geometry_msgs/Vector3 linear_acceleration
#  float64 x
#  float64 y
#  float64 z
#float64[9] linear_acceleration_covariance
def callback_imu_data(msg):
	data['imu'] = msg

# Demo functions
# turn left
def turn_left():
	try:
		linear_velocity = [0.0, 0.0, 0.0]
		angular_velocity = [0.0, 0.0, 0.4]
		return [linear_velocity, angular_velocity]
	except:
		raise ValueError("Lost connection to MiR in turn_left()")

# turn right
def turn_right():
	try:
		linear_velocity = [0.0, 0.0, 0.0]
		angular_velocity = [0.0, 0.0, -0.4]
		return [linear_velocity, angular_velocity]
	except:
		raise ValueError("Lost connection to MiR in turn_right()")

# forward
def forward():
	try:
		linear_velocity = [0.5, 0.0, 0.0]
		angular_velocity = [0.0, 0.0, 0.0]
		return [linear_velocity, angular_velocity]
	except:
		raise ValueError("Lost connection to MiR in forward()")

# backwards
def backwards():
	try:
		linear_velocity = [-1.5, 0.0, 0.0]
		angular_velocity = [0.0, 0.0, 0.0]
		return [linear_velocity, angular_velocity]
	except:
		raise ValueError("Lost connection to MiR in forward()")

def turn_right_full(self):
	try:
		start_yaw = getYawFast(self)
		thresh = start_yaw
		thresh += math.pi
		thresh -= math.pi / 2
		thresh %= 2 * math.pi
		thresh -= math.pi
		#print "start yaw:", start_yaw
		#print "threshold:", thresh

		while math.fabs( (getYawFast(robot)) - thresh ) > 0.27:
			move(self, turn_right()[0], turn_right()[1])
			#print "yaw:", getYawFast(robot)
			#print "thresh:", thresh
			#print "difference:", math.fabs(getYawFast(robot) - thresh)

	except:
		raise ValueError("Lost connection to MiR in turn_right_full()")

def turn_left_full(self):
	try:
		start_yaw = getYawFast(self)
		thresh = start_yaw
		thresh += math.pi
		thresh += math.pi / 2
		thresh %= 2 * math.pi
		thresh -= math.pi
		#print "start yaw:", start_yaw
		#print "threshold:", thresh

		while math.fabs( (getYawFast(robot)) - thresh ) > 0.27:
			move(self, turn_left()[0], turn_left()[1])
			#print "yaw:", getYawFast(robot)
			#print "thresh:", thresh
			print "difference:", math.fabs(getYawFast(robot) - thresh)
	except:
		raise ValueError("Lost connection to MiR in turn_left_full()")

# circle robot demo
def circle_demo(self,
				linear_vel = None,
				angular_vel= None,
				n_steps = None):
	try:
		if linear_vel == None:
			linear_vel = [0.2,0.0,0.0]
		if angular_vel == None:
			angular_vel = [0.0,0.0,-0.5]
		if n_steps == None:
			n_steps = 8000

		for i in range(n_steps):
			move(self, linear_vel, angular_vel)
	except:
		raise ValueError("Lost connection to MiR in circle_demo()")

#Square shape demo
def square_demo(self):
	try:
		for i in range(30):
			move(self, forward()[0], forward()[1])
			time.sleep(0.1)

		time.sleep(0.5)
		turn_right_full(self)
		time.sleep(0.5)

		for i in range(30):
			move(self, forward()[0], forward()[1])
			time.sleep(0.1)

		time.sleep(0.5)
		turn_right_full(self)
		time.sleep(0.5)

		for i in range(30):
			move(self, forward()[0], forward()[1])
			time.sleep(0.1)

		time.sleep(0.5)
		turn_right_full(self)
		time.sleep(0.5)

		for i in range(30):
			move(self, forward()[0], forward()[1])
			time.sleep(0.1)

		time.sleep(0.5)
		turn_right_full(self)
	except:
		raise ValueError("Lost connection to MiR in square_demo()")
# function for fast getting yaw from a Quaternion, when operating in 2D
# ranges between [0, 3.16]
def getYawFast(self):
	try:
		z = data['imu']['orientation']['z']
		#print z
		w = data['imu']['orientation']['w']
		#print z
		yaw = 2.0 * math.asin(z)
		if w*z < 0.0:
			yaw  = -math.fabs(yaw)

		return yaw
	except:
		raise ValueError("Lost connection to MiR in getYawFast")
