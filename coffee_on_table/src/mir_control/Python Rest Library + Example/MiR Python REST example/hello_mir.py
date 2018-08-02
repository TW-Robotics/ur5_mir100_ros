# Include MiR example library, which includes MiR REST library
from mir.mir import MiR

# Initialize MiR robot
robot = MiR("192.168.12.20")

# Example - MiR Communication

print("Registers:", robot.all_registers())
#print("Value of register 1:", robot.read_register(1))

#robot.write_register(1, 1)
#print("New value of register 1:", robot.read_register(1))
#robot.write_register(1, 42)
#print("Register 1:", robot.read_register("1"))

#print("Status:", robot.status_overview())
print("Robot Name:", robot.robot_name())
print("Battery Percentage:", robot.battery_percentage())

#robot.pause_robot()
#robot.continue_robot()
