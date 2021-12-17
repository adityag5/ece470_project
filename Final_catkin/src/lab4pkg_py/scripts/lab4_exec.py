#!/usr/bin/env python
import copy
import time
import rospy
import rospkg
import os
import sys
import numpy as np
import yaml
from math import pi
from lab4_header import *
from lab4_func import *
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import DeleteModel
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import random

# 20Hz
SPIN_RATE = 20

# UR3 home location
home = np.radians([120, -90, 90, -90, -90, 0])
Q1 = np.radians([265.56,-46.91,102.02,-138.23,-88.45,35.74])
Q2 = Q1 + np.radians([-180,0,0,0,0,0])

thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0
analog_in_1 = 0

suction_on = True
suction_off = False
current_io_0 = False
current_position_set = False
current_gripper_set = False

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)

#* Current ground position, relative to starting point
ground_position = Point(0,0,0)

#* Yaw value relative to the world frame in Euler Angles
ground_orientation = 0

"""
Define a ROS topic callback funtion for getting the state of suction cup
Whenever ur3/gripper_input publishes info this callback function is called.
"""
def quaternion_to_yaw(quaternions):

    w = quaternions[0]
    x = quaternions[1]
    y = quaternions[2]
    z = quaternions[3]

    yaw = math.atan2(2*(w*z + x*y),1 - 2*((y**2) + (z**2)))
    return yaw


def gripper_callback(msg):

    global digital_in_0
    global analog_in_0
    global analog_in_1
    global current_gripper_set

    digital_in_0 = msg.DIGIN
    analog_in_0 = msg.AIN0
    analog_in_1 = msg.AIN1

    current_gripper_set = True


"""
Whenever ur3/position publishes info, this callback function is called.
"""
def position_callback(msg):

    global thetas
    global current_position
    global current_position_set

    thetas[0] = msg.position[0]
    thetas[1] = msg.position[1]
    thetas[2] = msg.position[2]
    thetas[3] = msg.position[3]
    thetas[4] = msg.position[4]
    thetas[5] = msg.position[5]

    current_position[0] = thetas[0]
    current_position[1] = thetas[1]
    current_position[2] = thetas[2]
    current_position[3] = thetas[3]
    current_position[4] = thetas[4]
    current_position[5] = thetas[5]

    current_position_set = True

def groundpos_callback(msg):

    global ground_position
    global ground_orientation

    #* Get Pose of the Robot
    pose = msg.pose.pose 
    ground_position = pose.position
    #* Get orientation of the robot
    orientation = pose.orientation

    #* Convert to euler angle
    orientation_quaternions = [orientation.w,orientation.x,orientation.y,orientation.z]
    ground_orientation = quaternion_to_yaw(orientation_quaternions)

def gripper(pub_cmd, loop_rate, io_0):

    global SPIN_RATE
    global thetas
    global current_io_0
    global current_position

    error = 0
    spin_count = 0
    at_goal = 0

    current_io_0 = io_0

    driver_msg = command()
    driver_msg.destination = current_position
    driver_msg.v = 1.0
    driver_msg.a = 1.0
    driver_msg.io_0 = io_0
    pub_cmd.publish(driver_msg)

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error

def move_block(pub_cmd, loop_rate, loc, end):

    global Q
    global digital_in_0
    ja_loc = lab_invk(loc[0], loc[1], loc[2], loc[3])
    ja_loc_up = lab_invk(loc[0], loc[1], loc[2]+0.3, loc[3])
    ja_end_up = lab_invk(end[0], end[1], end[2]+0.3, end[3])
    ja_end = lab_invk(end[0], end[1], end[2] + 0.05, end[3])

    rospy.loginfo("Moving to: " + str(loc))

    move_arm(pub_cmd, loop_rate, ja_loc_up, 4.0, 4.0)
    move_arm(pub_cmd, loop_rate, ja_loc, 4.0, 4.0)

    rospy.loginfo("Gripper on")
    gripper(pub_cmd, loop_rate, suction_on)
    time.sleep(3.0)

    # Check if gripping
    if digital_in_0 == 0:
        error = 1
        rospy.loginfo("NO OBJECT DETECTED")
        gripper(pub_cmd, loop_rate, suction_off)
        return error
    else:
        rospy.loginfo("BLOCK DETECTED")

    move_arm(pub_cmd, loop_rate, ja_end_up, 4.0, 4.0)
    move_arm(pub_cmd, loop_rate, ja_end, 4.0, 4.0)

    time.sleep(0.5)
    rospy.loginfo("Gripper off")
    gripper(pub_cmd, loop_rate, suction_off)
    time.sleep(0.5)

    error = 0
    return error

def move_arm(pub_cmd, loop_rate, angles, vel, accel):    
    global thetas
    global SPIN_RATE

    error = 0
    spin_count = 0
    at_goal = 0

    driver_msg = command()
    driver_msg.destination = angles
    driver_msg.v = vel
    driver_msg.a = accel
    driver_msg.io_0 = current_io_0
    pub_cmd.publish(driver_msg)

    loop_rate.sleep()

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error

def calc_offsets(yaw):
        theta = random.uniform(-1 * pi / 2,pi/2) +  yaw

        R_spawn = 0.35

        #* Offsets in Space Frame
        x_offset = R_spawn * np.cos(theta)
        y_offset = R_spawn * np.sin(theta)

        return x_offset, y_offset, theta, R_spawn

def xy_dist(x1,y1,x2,y2):
    return np.sqrt((x2 - x1)**2 + (y2 - y1)**2)


def main():

    global home
    global Q
    global SPIN_RATE

    # Initialize ROS node
    rospy.init_node('lab2node')

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

    # Initialize subscriber to ur3/position and callback fuction
    # each time data is published
    sub_position = rospy.Subscriber('ur3/position', position, position_callback)

    # define a ROS subscriber for ur3/gripper_input message and corresponding callback function
    sub_gripper = rospy.Subscriber('ur3/gripper_input', gripper_input, gripper_callback)

    # Position subscriber
    sub_groundpos = rospy.Subscriber('/odom', Odometry, groundpos_callback)

    # Initialize RosPack and node to spawn blocks
    rospack = rospkg.RosPack()
    
    # Get path to blocks
    block_path = rospack.get_path('ur_description')
    block_path_1 = os.path.join(block_path,'urdf','block_spawn.urdf')
    block_path_2 = os.path.join(block_path,'urdf','block_spawn2.urdf')

    # Wait for service
    rospy.wait_for_service('gazebo/spawn_urdf_model')

    # Create services
    spawn = rospy.ServiceProxy('gazebo/spawn_urdf_model', SpawnModel)
    delete = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)

    # Check if ROS is ready for operation
    while(rospy.is_shutdown()):
        print("ROS is shutdown!")

    rospy.loginfo("Sending Goals ...")
    loop_rate = rospy.Rate(SPIN_RATE)

    cart_move = rospy.Publisher('/cmd_vel',Twist,queue_size=7)

    #* World Destination for robot to move to, provided by user through the shell
    w_des = [float(sys.argv[1]),float(sys.argv[2])]

    #* Calculate the angle to move at 
    movement_angle = math.atan2(w_des[1],w_des[0])

    #* Create movement message
    movement = Twist()

    #* Start Timer
    start_time = time.time()

    #* Turn the car
    while (abs(ground_orientation - movement_angle) > 0.001):
        movement.linear.x = 0.00
        movement.linear.y = 0.00

        if (abs(ground_orientation - movement_angle) > 0.01):
            movement.angular.z = 0.075 * np.sign(movement_angle)
        else: #Slower, finer turning when closer to the desired angle
            movement.angular.z = 0.0075 * np.sign(movement_angle)
        cart_move.publish(movement)

    #* Reset all velocity to 0
    movement.linear.x = 0.00
    movement.linear.y = 0.00
    movement.angular.z = 0.00
    cart_move.publish(movement)

    #* Flag for if we are at next block
    at_block = 0

    #* Current block number
    block_num = 0

    #* Yaw to Quaternion Conversion, used when spawning blocks
    qw = np.cos(movement_angle/2) 
    qx = 0
    qy = 0
    qz = np.sin(movement_angle/2) 

    #* Flag to check if we are at the destination
    at_des = 0

    #* Flag to check if we failed to get close enough to the destination
    failed = 0

    #* Minimum Error between Destination and Endpoint
    min_error = [10000,10000]
    min_error_magnitude = xy_dist(min_error[0],min_error[1],0,0)

    #* Error List
    error_list = [min_error_magnitude]

    while (True):
        #* Tolerance for how close blocks can spawn together
        tol = 0.3

        #* Values for 1st block
        x_offset_1, y_offset_1, theta1,R1 = calc_offsets(movement_angle)

        #* Spawn 1st Box
        starting_pose = Pose(Point(ground_position.x + x_offset_1, ground_position.y + y_offset_1, 0.01), Quaternion(qx,qy,qz,qw))
        spawn("Block" + "_" + str(block_num), open(block_path_1, 'r').read(), 'block', starting_pose, 'world')
        block_num += 1

        #* Values for 2nd block
        x_offset_2, y_offset_2, theta2,R2 = calc_offsets(movement_angle) 
        dist_1_2 = xy_dist(x_offset_1,y_offset_1,x_offset_2,y_offset_2)
        while (dist_1_2 < tol):
            x_offset_2, y_offset_2, theta2,R2 = calc_offsets(movement_angle)
            dist_1_2 = xy_dist(x_offset_1,y_offset_1,x_offset_2,y_offset_2)

        #* Spawn 2nd Box
        starting_pose = Pose(Point(ground_position.x + x_offset_2, ground_position.y + y_offset_2, 0.01), Quaternion(qx,qy,qz,qw))
        spawn("Block" + "_" + str(block_num), open(block_path_2, 'r').read(), 'block', starting_pose, 'world')
        block_num += 1

        #* Where the robot will place the block with respect to itself(constant)
        pos_end = [0.15, 0.15, -0.05, 0]
        dist_from_cart = 0.15 + pos_end[0] # In robots x direction

        #* Decide which box to move
        x_pos = dist_from_cart * np.cos(movement_angle)
        y_pos = dist_from_cart * np.sin(movement_angle) 
        dist1 = xy_dist(x_offset_1,y_offset_1,x_pos,y_pos)
        dist2 = xy_dist(x_offset_2,y_offset_2,x_pos,y_pos)
        dist_array = np.array([dist1,dist2])

        R = 0
        theta_spawn = 0
        x_offset = 0
        y_offset = 0

        time.sleep(1)
        print("CHOOSING BLOCK")

        min_idx = np.argmin(dist_array)

        theta_spawn = 0
        if (min_idx == 0): #Block 1 is closest
            theta_spawn = theta1
            R = R1
            delete("Block" + "_" + str(block_num - 1))
        else: #Block 2 is closest
            theta_spawn = theta2
            R = R2
            delete("Block" + "_" + str(block_num - 2))

        #* Position of the box spawned with respect to the robot
        x_box_robot = R*np.sin(pi/2 - movement_angle + theta_spawn)
        y_box_robot = R*np.cos(pi/2 - movement_angle + theta_spawn)

        #* Move the block
        pos_box = [x_box_robot - 0.15,-(y_box_robot - 0.15),-0.075,0]
        status = move_block(pub_command, loop_rate, pos_box, pos_end)
        move_arm(pub_command, loop_rate, home, 4.0, 4.0)
        time.sleep(0.1)

        #* Position of the block thats been placed wrt the cart
        dist_from_cart = 0.15 + pos_end[0]
        #* Target destination of the cart in the world frame
        x_pos = dist_from_cart * np.cos(movement_angle)
        y_pos = dist_from_cart * np.sin(movement_angle) 
        target_dest = np.add([ground_position.x,ground_position.y, ground_position.z,0],[x_pos, y_pos,0,0])

        #* Move cart until its at the target
        movement.linear.x = 0.03
        movement.angular.z = 0.0
        movement.angular.y = 0.0
        cart_move.publish(movement)

        while(at_block == 0 and at_des == 0 and failed == 0):
            #*Calculate Error between current position and endpoint 
            error_world = [abs(ground_position.x-w_des[0]),abs(ground_position.y-w_des[1])]
            error_magnitude = xy_dist(ground_position.x,ground_position.y,w_des[0],w_des[1])

            #* Calculate Error between current position and next block
            error_block = [abs(ground_position.x-target_dest[0]),abs(ground_position.y-target_dest[1])]

            #* Update minimum error
            if (error_magnitude < min_error_magnitude):
                min_error_magnitude = error_magnitude
                min_error_ = error_world

            print("ERROR: ", error_world)

            if (error_block[0] < 0.003 and error_block[1] < 0.003):
                at_block = 1

            if (error_magnitude < 0.08):
                at_des = 1

            if (error_magnitude > error_list[-1]):
                failed = 1        
            
            error_list.append(error_magnitude)

            loop_rate.sleep()

        #* Check if we reached the destination or failed to get to it
        if (at_des == 1):
            print("REACHED DESTINATION")
            break
        elif (failed == 1):
            print("DID NOT REACH DESTINATION")
            break
            
        #* Stop car from moving
        movement.linear.x = 0.0
        movement.angular.z = 0.0
        movement.angular.y = 0.0
        cart_move.publish(movement)

        #* Update Flag
        at_block = 0

    end_time = time.time()

    print("TIME TAKEN: ", end_time - start_time)
    print("DESTINATION ERROR: ",abs(ground_position.x-w_des[0]),abs(ground_position.y-w_des[1]))
    print("Z Position: ", ground_position.z )

    #* Stop the car from moving
    movement.linear.x = 0.0
    movement.linear.y = 0.0
    movement.angular.z = 0.0
    cart_move.publish(movement)

if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass
