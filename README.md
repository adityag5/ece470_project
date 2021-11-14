# ECE 470 Project Update 2 #

### catkin_ws ###

This is the main catkin workspace for this project. It contains the drivers for the robot, the source files and the Python scripts we used to complete the pick and place task with the UR3 arm.  

- src
 - lab4pkg_py
  - scripts
   - lab4_exec.py - This file contains functions which enable movement in the robot arm and exectues the commands to pick up the block
   - lab4_func.py - This file contains the code to compute inverse and forward kinematics
   - spawn.py - This file spawns blocks in the Gazebo world.

### skid4wd_ws ###

This it the catkin workspace used for building and moving the model for the cart. In the future, this directory is to be deleted as catkin_ws will contain the implementation for the moving cart integrated with the UR3 arm on it.


