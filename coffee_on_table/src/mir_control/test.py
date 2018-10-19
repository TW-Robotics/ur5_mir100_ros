# -*- coding: utf-8 -*-
import rosbridge_2_0 as rb
from rosbridge_2_0 import time
from rosbridge_2_0 import json

# robot url
robot = rb.boot()


# wait for pose data - do not uncomment
while(len(rb.data['pose'])) == 0:
    print "waiting for pos - 0.5s"
    time.sleep(0.5)
    
# Movement
'''
linear_velocity = [0.5, 0.0, 0.0]
angular_velocity = [0.0, 0.0, 0.0]
for i in range(1000):
    rb.move(robot, linear_velocity, angular_velocity)

linear_velocity = [1, 0.0, 0.0]
angular_velocity = [0.0, 0.0, 0.0]
for i in range(1000):
    rb.move(robot, linear_velocity, angular_velocity)
'''
# Simple movement patterns

#for i in range(1000):
rb.move(robot, rb.forward()[0], rb.forward()[1])
#time.sleep(2)
#for i in range(1000):
#    rb.move(robot, rb.forward()[0], rb.forward()[1])
#time.sleep(2)
#for i in range(1000):
rb.moveToGoal(robot)
#time.sleep(2)
    #rb.move(robot, rb.backwards()[0], rb.backwards()[1])
'''time.sleep(2)
for i in range(1000):
    rb.move(robot, rb.turn_left()[0], rb.turn_left()[1])
time.sleep(2)
for i in range(1000):
    rb.move(robot, rb.turn_right()[0], rb.turn_right()[1])
'''
#for i in range(100):
#    rb.move(robot, rb.backwards()[0], rb.backwards()[1])


# Accessing and printing robot data

#for i in range(100):
#    rb.move(robot, rb.turn_right()[0], rb.turn_right()[1])
#print json.dumps(rb.data['pose'], indent = 2)
print json.dumps(rb.data['goal'], indent = 2)
print json.dumps(rb.data['velocity'], indent = 2)


# Rosservice
'''
message = {"robotMode": 7}
print robot.callService("/mircontrol/setMode", msg = message)
'''

# Circle demo
'''
rb.circle_demo(robot)
'''

# Square demo

#rb.square_demo(robot)

