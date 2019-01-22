import sys
import rospy

''' Class handles goals of MiR '''

class goal():
	# Init empty goal
	def __init__(self, goalName):
		self.name = "invalid"
		self.posx = 0 		# meters
		self.posy = 0 		# meters
		self.rz = 0 		# degrees
		self.height = 0 	# mm
		self.orientation = "invalid"	# left, right or front
		
		# Fill goal with useful information
		self.createGoal(goalName)

	# Check if goal-name exists and make goal
	def createGoal(self, goalName):
		if goalName == "table1":
			self.name = "Table at MiR Charger"
			self.posx = 5.12
			self.posy = 4.11
			self.rz = 87.4
			self.height = 720
			self.orientation = "right"
		elif goalName == "table2":
			self.name = "Table in front of workshop"
			self.posx = 14.35
			self.posy = 8.9
			self.rz = 0
			self.height = 720
			self.orientation = "left"
		elif goalName == "robot1":
			self.name = "Robot workstation 1"
			self.posx = 8.7
			self.posy = 11.9
			self.rz = 100
			self.height = 990
			self.orientation = "front"
		return self

	# Print information of goal
	def display(self):
		print " Goal: ", self.name, "Height: ", str(self.height), "mm"
		print "  MIR: ", str(self.posx), str(self.posy), str(self.rz)
		print "   UR: ", self.orientation

def main(args):
	newm = goal("table1")
	newm.display()

if __name__ == '__main__':
	main(sys.argv)
