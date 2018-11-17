import socket
import time

HOST = "192.168.12.52"    	# The remote host
PORT = 30002              	# The same port as used by the server

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))
s.send ("set_digital_out(2,True)" + "\n")
time.sleep (2)
s.send ("set_digital_out(2,False)" + "\n")
time.sleep (1)

#s.send ("popup(\"Messages\", title=\"The Headline in the Blue box\", blocking=True)" + "\n")
#time.sleep (1)

#s.send ("set_digital_out(2,False)" + "\n")
#time.sleep (1)

data = s.recv(1024)

print "Good bye!"