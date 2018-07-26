#!/usr/bin/env python
import time
import roslib; #roslib.load_manifest('ur_driver')
import rospy
import sys
import actionlib
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from control_msgs.msg import *
from trajectory_msgs.msg import *
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from math import pi

#Source: http://docs.ros.org/kinetic/api/moveit_commander/html/move__group_8py_source.html
#More doku: http://docs.ros.org/kinetic/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroupInterface.html

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

Q1 = [-1.13,   -1.56, -0.65,   -0.94, 1.81, 0.54]
Q2 = [-1.38,   -1.56, -0.65,   -0.94, 1.81, 0.54]
Q3 = Q1

client = None  
def move_repeated():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    try:
        joint_states = rospy.wait_for_message("joint_states", JointState)
        joints_pos = joint_states.position
        d = 2.0
        g.trajectory.points = [JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0))]
        for i in range(10):
            g.trajectory.points.append(
                JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(d)))
            d += 1
            g.trajectory.points.append(
                JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(d)))
            d += 1
            g.trajectory.points.append(
                JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(d)))
            d += 2
        client.send_goal(g)
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise
    except:
        raise

checkBeforeDo = False

class MoveGroupPythonInterface_ur5(object):
    def __init__(self):
        super(MoveGroupPythonInterface_ur5, self).__init__()
        self.speed = 0.1
        self.acceleration = 0.1

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface', anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        group_name = "manipulator"
        self.group = moveit_commander.MoveGroupCommander(group_name)

    def moveRobot(self, goal_pose):
        print " ******************** Goal Pose **"
        print goal_pose

        self.group.set_pose_target(goal_pose)
        plan = self.group.plan()
		
		#self.group.retime_trajectory( ,plan, 0.2)
        self.group.set_max_velocity_scaling_factor(self.speed)
        self.group.set_max_acceleration_scaling_factor(self.acceleration)

        print " Visualizing path in RViz..."
        #rospy.sleep(5)

        inp = ""

        if (checkBeforeDo):
            inp = raw_input("Move robot as visualized? y/n: ")[0]
        if (inp == 'y' or checkBeforeDo == False):
            self.group.go(wait=True)
            self.group.stop()
            self.group.clear_pose_targets()

        current_pose = self.group.get_current_pose().pose
        print " ******************** New Pose **"
        print current_pose

    def move_to_pose(self, goalPose):
        goal_pose = geometry_msgs.msg.Pose() # Point x, y, z in Meter; Orientation x, y, z, w in Quaternionen
        current_pose = self.group.get_current_pose().pose
		
        print " ******************** Current Pose **"
        print current_pose
        
        goal_pose.position.x = goalPose[0]
        goal_pose.position.y = goalPose[1]
        goal_pose.position.z = goalPose[2]
        goal_pose.orientation.x = goalPose[3]
        goal_pose.orientation.y = goalPose[4]
        goal_pose.orientation.y = goalPose[5]
        goal_pose.orientation.w = goalPose[6]

        self.moveRobot(goal_pose)

    def move_xyz(self, x, y, z):
        current_pose = self.group.get_current_pose().pose
        goal_pose = current_pose
        print " ******************** Current Pose **"
        print current_pose

        goal_pose.position.x = current_pose.position.x + x
        goal_pose.position.y = current_pose.position.y + y
        goal_pose.position.z = current_pose.position.z + z

        self.moveRobot(goal_pose)

    def move_to_jointStates(self, jointStates):
        current_jointStates = self.group.get_current_joint_values()
        goal_jointStates = current_jointStates
        print " ******************** Current Pose **"
        print current_jointStates

        goal_jointStates = jointStates

        print " ******************** Goal Pose **"
        print goal_jointStates

        self.group.set_max_velocity_scaling_factor(self.speed)
        self.group.set_max_acceleration_scaling_factor(self.acceleration)

        inp = ""

        if (checkBeforeDo):
            inp = raw_input("Move robot? y/n: ")[0]
        if (inp == 'y' or checkBeforeDo == False):
            self.group.go(goal_jointStates, wait=True)
            self.group.stop()

    def go_home(self):
        # Home: 0.005937059875577688, -1.5655563513385218, -0.00637227693666631, -1.5696209112750452, 0.009078050963580608, 0.01515068206936121]
        goalPose = [0, -1.565, 0, -1.569, 0, 0]
        self.move_to_jointStates(goalPose)

    def plan_cartesian_path(self, scale=1):
        waypoints = []

        wpose = self.group.get_current_pose().pose
        wpose.position.z += scale * 0.1  # First move down (z)
        #wpose.position.y += scale * 0.02  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        #wpose.position.x += scale * 0.01  # Second move forward/backwards in (x)
        #waypoints.append(copy.deepcopy(wpose))

        #wpose.position.y -= scale * 0.01  # Third move sideways (y)
        #waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
        (plan, fraction) = self.group.compute_cartesian_path(
                                           waypoints,   # waypoints to follow
                                           0.01,        # eef_step
                                           0.0)         # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan

    def execute_plan(self, plan):
        plan = self.group.retime_trajectory(self.robot.get_current_state(), plan, 0.05)

        inp = ""

        if (checkBeforeDo):
            inp = raw_input("Move robot as visualized? y/n: ")[0]
        if (inp == 'y' or checkBeforeDo == False):
            self.group.execute(plan, wait=True)
		
def main():
    '''global client
    try:
        rospy.init_node("test_move", anonymous=True, disable_signals=True)
        client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for server..."
        client.wait_for_server()
        print "Connected to server"
        parameters = rospy.get_param(None)
        index = str(parameters).find('prefix')
        if (index > 0):
            prefix = str(parameters)[index+len("prefix': '"):(index+len("prefix': '")+str(parameters)[index+len("prefix': '"):-1].find("'"))]
            for i, name in enumerate(JOINT_NAMES):
                JOINT_NAMES[i] = prefix + name
        print "This program makes the robot move"
        #print str([Q1[i]*180./pi for i in xrange(0,6)])
        #print str([Q2[i]*180./pi for i in xrange(0,6)])
        #print str([Q3[i]*180./pi for i in xrange(0,6)])
        print "Please make sure that your robot can move freely before proceeding!"
        inp = raw_input("Continue? y/n: ")[0]
        if (inp == 'y'):
            #move1()
            moveXYZ()
            #move_repeated()
            #move_disordered()
            #move_interrupt()
        else:
            print "Halting program"'''
    try:
        ur5 = MoveGroupPythonInterface_ur5()
        print "Planning path to goal position (check at rviz!)"
        
        ur5.go_home()
        
        goalPose = [0, 0.191, 0.937, 0.707, 0, 0, 0.707]
        #goalPose = [0, 0.191, 0.937, 0, 0, 0, 0.707]
        # Get actual pose: rosrun tf tf_echo base_link tool0
        
        ur5.move_to_pose(goalPose) # x, y, z, rx, ry, rz, w

        x_inc = 0
        y_inc = 0.1
        z_inc = -0.1
        
        ur5.move_xyz(x_inc, y_inc, z_inc)
        
        jointStates = [-1.13, -1.56, -0.65, -0.94, 1.81, 0.54]
        
        ur5.move_to_jointStates(jointStates)
        
        plan = ur5.plan_cartesian_path()
        ur5.execute_plan(plan)
        
        ur5.go_home()

    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__':
    main(sys.argv)
