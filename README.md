# ECE 470 Project #

### project1.py ### 
This Python file contains the basic implementation of our project update 1. It makes the end-effector go to a specified point to the front of the robot, activate the gripper and then depending on whether there is a block detected using the gripper sensor, it will pick it up and then move to a point to the back of the robot and then release it. This motion allows the robot arm to lay a path for itself. It should be in the lab2andDriver environment.

### ScreenRecordingRobot.mp4 ### 
[This is the video](https://youtu.be/YgwciGULasQ) demonstrating the UR3 responding to project1.py. The robot arm in Gazebo can be seen moving to the first waypoint, trying to pick up a block, sensing no block there, moving to the next waypoint, trying to pickup a block, sensing no block. The console can be seen to verify that a console log is created indicating that blocks were not detected by the gripper.

### Console-Output ###
This is a text file to give a clearer look at the console logs that occur during the robot's movements.

### catkin_ws###

This is the main catkin workspace for this project. It contains the drivers for the robot, the source files and the Python scripts we used to complete the pick and place task with the UR3 arm.
