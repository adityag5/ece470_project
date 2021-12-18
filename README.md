# ECE 470 Final Project Update #

### Final_catkin ###

This is the final updated catkin workspace for the project. It contains the drivers for the robot, the source files and the Python scripts we used to complete the pick and place task with the UR3 arm.  

To launch the Gazebo simulation environment, the user can run the following command within the workspace:
```
roslaunch ur3_driver ur3_gazebo.launch
```

* src/lab4pkg_py/scripts
  * lab4_exec.py - This file contains functions which enable movement in the robot arm and exectues the commands to both move the cart and pick up blocks. It also spawns blocks in the Gazebo simulator
  * lab4_func.py - This file contains the code to compute inverse and forward kinematics

The lab4_exec.py script is run with 2 arguments. The first is the desired x waypoint, and the second is the desired y waypoint. For example, if the user wants the robot/cart to move to an x position of 3 and a y position of 1 in the Gazebo world frame, the user would input:
```
rosrun lab4pkg_py lab4_exec.py 3 1
```
* lab2andDriver/drivers/universal_robot/ur_description
  * movingbox.urdf - This file contains the urdf model for the cart and the planar move plugin that allows for control and pose subscription of the cart 
  * block_spawn.urdf and block_spawn2.urdf - These contains models of the blocks that are spawned for picking and placing
  * ur3_robot.urdf.xacro - This contains the joint model of the robot and the cart
* lab2andDriver/drivers/universal_robot/ur3_driver
  * ur3_gazebo.launch - This is the launch file used to create the simulation environment. It spawns the robot on top of a block on the ground. 

