import sys
import rospy
import xml.etree.ElementTree as ET

class missionGoal(object):
	def __init__(self):
		# Init empty goal
		self.name = "none"
		self.posx = 0
		self.posy = 0
		self.rz = 0
		self.orientation = "none"

	def display(self):
		print "Goal: ", self.name
		print "MIR: ", str(self.posx), str(self.posy), str(self.rz)
		print "UR: ", self.orientation

class parser(object):
	#def __init__(self):

	def parseGoals(self, path):
		tree = ET.parse("xml/" + path)
		root =  tree.getroot()

		#numGoals = len(root)
		goals = []

		for goal in root.iter('goal'):
			goalX = missionGoal()
			goalX.name = goal.attrib["name"]
			goalX.posx = goal.find("posx").text
			goalX.posy = goal.find("posy").text
			goalX.rz = goal.find("rz").text
			goalX.orientation = goal.find("UROrientation").text
			goals.append(goalX)

		return goals

def main(args):
	goals = parser().parseGoals("goals.xml")
	for goal in goals:
		goal.display()
	return

if __name__ == '__main__':
	main(sys.argv)
