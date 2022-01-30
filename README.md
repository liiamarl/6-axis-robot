# 6-axis-robot
model of a 6 axis industrial robot arm using dynamixel servos.

It uses 6 dynamixel servos:
  - AX12A for joints 1 and 3
  - XC430-W240-T for joint 2
  - XL330-M288-T for joints 4, 5 & 6
servos 1, 2 and 3 are connected in a first bus, servos 4, 5 and 6 in a second one.
the software runs on a raspberry Pi and there are two U2D2 communication converters to communicate with the servos buses. 
there are a 12V 10A and 5V 10A power supplys (little bit overkill but they are super cheap).

I made all of it myself, just using a few python libraries for linear algebra (numpy) and communication protocol with the servos (DynamixelSDK).
I wanted to make the software myself in oder to learn as much as possible from this project, but i will eventually use ROS2 since it is way better than anything i could write myself.

For now it is able to repeat a program taught by manually moving it and can be moved in the world frame from keyboard inputs. It can also be taught user frames using the 3 points method and TCP offset using the 4 points method (i understand how the 3 points method works but didn't implement it yet)

I'm not trully satisfied with the design for a few reasons :
  - I didn't really planned cable management, so wiring is messy, particularry around the wrist.
  - The distance between joint 3 and the end plate of the robot is to large compared to the distance between joints 2 and 3, witch limits the robot's movements.
  - The AX-12A servos don't have PID control, and that's the main reason why the movements smoothness and precision are not as good as they could be (also software   issues but that can be resolved).
Therefore i will remake it when i'm satisfied with my software and managed to control it with ROS2. I'm planning on using only XL330-M288-T , with a bit of reduction for the first 3 joints (backclash shouldn't be an issue on joints 2 and 3 due to preload). This would allow for removal of the 12V power supply and a much simpler electrical design. 

Some videos of the robot in action (change the quality to 1080p otherwise it's all blurry idk what append here): 
speed testing : 
https://drive.google.com/file/d/13-3ETRIs8skaFkm4ivGpO7MBdzcWkttJ/view?usp=sharing

manual teach and repeat :
https://drive.google.com/file/d/13KtLAo7f85VwjT2PGi888gaMC-sLZVy7/view?usp=sharing

world frame movements with a 90g payload (movements are not smooth but the point was only to test the inverse kinematics):
https://drive.google.com/file/d/13RQ4p_W5TPF8WYblhjcw3lXpy9PoeAgZ/view?usp=sharing

You can find the electricals schematics and the source code in this repository, keep in mind this is still far from finished :)


