import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../Server'))

from servo import Servo
import time
# Initialize the servo controller
my_servos = Servo()
print("Setting servos to their default starting positions...")
# Set Servo '0' (The Gripper) to exactly 90 degrees (Open position)
my_servos.setServoAngle('0', 90)
# Set Servo '1' (The Lift Arm) to exactly 90 degrees (Down position)
my_servos.setServoAngle('1', 90)
time.sleep(2)
print("Servos are locked at 90 degrees. You can now attach the arm parts!")
