import socket
import time
import sys

HOST = "192.168.12.52"    	# The remote host
PORT = 30002              	# The same port as used by the server

def open():
	print "Opening Gripper"
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.connect((HOST, PORT))
	s.send ("set_digital_out(2,True)" + "\n")
	print "Open-Gripper Command sent"

def close():
	print "Closing Gripper"
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.connect((HOST, PORT))
	s.send ("set_digital_out(3,True)" + "\n")
	print "Close-Gripper Command sent"

def main(args):
	print "Unable to use package this way!"
	return

if __name__ == '__main__':
	main(sys.argv)
