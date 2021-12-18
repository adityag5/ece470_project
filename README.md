# ECE 470 Final Project Update #

### Final_catkin ###

This is the final updated catkin workspace for the project. It contains the drivers for the robot, the source files and the Python scripts we used to complete the pick and place task with the UR3 arm.  

* src/lab4pkg_py/scripts
  * lab4_exec.py - This file contains functions which enable movement in the robot arm and exectues the commands to both move the cart and pick up blocks. It also spawns blocks in the Gazebo simulator
  * lab4_func.py - This file contains the code to compute inverse and forward kinematics

The lab4_exec.py script is run with 2 arguments. The first is the desired x waypoint, and the second is the desired y waypoint. For example, if the user wants the robot/cart to move to an x position of 3, and a y position of 1, the user would input:

rosrun lab4pkg_py lab4_exec.py 3 1

* lab2andDriver/drivers/universal_robot/ur_description
  * movingbox.urdf - This file contains the urdf model for the cart and the planar move plugin that allows for control and pose subscription of the cart 





# ECE 470 Project Update 2 #

### catkin_ws ###

This is the main catkin workspace for this project. It contains the drivers for the robot, the source files and the Python scripts we used to complete the pick and place task with the UR3 arm.  

* src/lab4pkg_py/scripts
  * lab4_exec.py - This file contains functions which enable movement in the robot arm and exectues the commands to pick up the block
  * lab4_func.py - This file contains the code to compute inverse and forward kinematics
  * spawn.py - This file spawns blocks in the Gazebo world.

### skid4wd_ws ###

This it the catkin workspace used for building and moving the model for the cart. In the future, this directory is to be deleted as catkin_ws will contain the implementation for the moving cart integrated with the UR3 arm on it.




