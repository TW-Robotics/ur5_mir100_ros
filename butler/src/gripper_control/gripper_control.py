import socket
import time
import sys
import rospy

from ur_msgs.msg import IOStates

''' Connects to the gripper via the UR to open/close it and check if grasped successfully'''

HOST = "192.168.12.52"    	# The remote host
PORT = 30002              	# The same port as used by the server

class gripper(object):
	def __init__(self):
		rospy.Subscriber('/ur_driver/io_states', IOStates, self.iostatesCallback)
		self.ioStates = IOStates()

	# Get IO-states to check if grasp has been successful
	def iostatesCallback(self, data):
		self.ioStates = data

	# Program on UR5 sets DO4 = True if grasped successfully
	def hasGripped(self):
		if self.ioStates.digital_out_states[4].state == True:
			return True
		else:
			return False

	# Send command to open the gripper (DO2)
	def open(self):
		print "Opening Gripper"
		s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		s.connect((HOST, PORT))
		s.send ("set_digital_out(2,True)" + "\n")
		print "Open-Gripper command sent."

	# Send command to close the gripper (DO3)
	def close(self):
		print "Closing Gripper"
		s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		s.connect((HOST, PORT))
		s.send ("set_digital_out(3,True)" + "\n")
		print "Close-Gripper command sent."

def main(args):
	print "Unable to use package this way!"
	return

if __name__ == '__main__':
	main(sys.argv)
