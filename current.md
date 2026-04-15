Step 1: Disconnect the Hardware
Before running any code, the servo motors must be able to spin freely without hitting any
plastic parts.
1. Make sure your Raspberry Pi is turned OFF.
2. Carefully unscrew the small screws holding the white plastic servo horns (the pieces
attached to the moving arm and claw).
3. Pull the plastic arm and claw pieces off the metal/plastic gears of the servos.
4. Turn your Raspberry Pi ON.
Step 2: The Software Calibration
We need to use a short Python script to force the servos to lock into their default "Starting"
positions (90 degrees).
1. On your Raspberry Pi, open your code editor and create a new file named calibrate.py.
2. Copy and paste the following code into the file:
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
3. Run the script. You will hear the servos buzz briefly as they snap to the 90-degree
position. Leave the Raspberry Pi powered ON so the servos hold this position.
Step 3: Physical Assembly (Locking the Angles)
While the Pi is still running and the motors are locked at 90 degrees, you will reattach the plastic
pieces to match your code's logic.
● Servo '1' (The Lift Arm): Take the main lifting arm and press it onto the servo gear so that
the arm is pointing straight forward, resting in its lowest possible "DOWN" position. Put
the screw back in.
● Servo '0' (The Gripper/Claw): Take the claw mechanism and press it onto the servo gear
so that the claws are completely "OPEN". Put the screw back in.
Step 4: Fine-Tuning Your Main Code
Now that the hardware is synced to the 90-degree baseline, you can safely test your
mode_clamp_up() and mode_clamp_down() functions in your main Car class.
Depending on the size of the objects you are grabbing, you may need to tweak the loops in
your code:
● If the claw crushes the object: The closing angle is too tight. Find the line for i in
range(90, 130, 1): for Servo '0' and lower the maximum angle (e.g., change 130 to 115).
● If the claw drops the object: The grip is too loose. Increase the closing angle (e.g.,
change 130 to 140).
● If the arm doesn't lift high enough: Find the line for i in range(90, 140, 1): for Servo '1' and
increase the 140 value slightly