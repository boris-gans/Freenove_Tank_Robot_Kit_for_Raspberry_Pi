Welcome to the ultimate test of your robotics engineering and programming skills! For your final project, you will deploy your Freenove 4WD Raspberry Pi tank as an automated logistics robot on a simulated factory floor.

Instead of separate tasks, your robot must run a single, continuous loop. It will need to prioritize sensor data, switch behaviors on the fly, and complete a comprehensive fetch-and-deliver mission completely autonomously.

🏭 The Scenario: The Automated Factory Floor
The arena consists of a main "highway" (a black line circuit), unexpected debris (obstacles on the path), scattered cargo (red balls), and a central "Drop Zone" circle.

The Objectives in Sequence:
Navigate the Highway: The robot must continuously follow the black line track around the arena using its downward IR sensors.
Dodge the Debris: Obstacles will be placed directly on the black line. If the front-facing ultrasonic sensor detects an obstacle, the robot must safely maneuver around it and re-acquire the black line to continue its path.
Locate and Retrieve Cargo: While navigating the course, the robot must use its camera to scan for a red ball placed somewhere along the route. Once the red ball is spotted, the robot must leave the line, approach the ball, and use its robotic arm to securely grab it.
Deliver to the Center: Once the payload is secured, the robot must navigate to the large circle in the center of the arena and autonomously release the ball into the Drop Zone.
⚖️ Competition Rules & Constraints
Full Autonomy: The robot must execute the entire sequence from a single Python script. Once you press "run," hands off! No remote controls, keyboards, or manual triggering of the arm.
Prioritization (The Main Loop): Your code must successfully balance sensor inputs. For example, if the robot is following a line but sees the red ball, the vision logic must override the line-tracking logic.
Hardware Restrictions: You may only use the components provided in the standard Freenove 4WD kit.
Interventions: If your robot gets completely stuck, crashes, or drops the ball, you may call an "Intervention." You can reset the robot back to the starting line, but you will incur a +30 second penalty.
🏆 Scoring System
This is a time-and-points-based challenge. Teams are ranked by total points, with the fastest completion time serving as the primary tie-breaker.

Points Awarded For:

+10 Points: Successfully avoiding an obstacle and re-acquiring the line track.
+20 Points: Successfully identifying the red ball and grabbing it with the robotic arm.
+20 Points: Successfully dropping the red ball entirely within the central Drop Zone circle.
Penalties:

-5 Points: Physical collision with an obstacle (your robot must gently evade, not ram it!).
-10 Points: Dropping the ball while transporting it.
🛠️ Pro-Tips for the Grand Challenge
State Machines are your Friend: Don't write one massive, messy while loop. Divide your code into "states" (e.g., STATE = "FOLLOW_LINE", STATE = "EVADE", STATE = "FETCH"). Let your sensor data dictate which state is currently active.
Vision Processing is Heavy: Running OpenCV on a Raspberry Pi can slow down your frame rate and your motor response times. Optimize your code to only scan for the red ball every few frames, or downscale the camera resolution to keep the robot moving smoothly.
Calibrate Everything: Arrive early on competition day! The room lighting will affect your color masking for the red ball, and the floor surface might affect how quickly your motors turn during an obstacle evasion maneuver.