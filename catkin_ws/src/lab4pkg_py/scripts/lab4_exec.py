#!/usr/bin/env python

# import os
# import argparse
import copy
import time
import rospy
import rospkg
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

# 20Hz
SPIN_RATE = 20

av = 2

# UR3 home location
home = np.radians([120, -90, 90, -90, -90, 0])

# Project 1

Q1 = np.radians([265.56,-46.91,102.02,-138.23,-88.45,35.74])
#Q1 = np.radians([85.56,-46.91,102.72,-57.6,-88.45,35.74])
Q2 = Q1 + np.radians([-180,0,0,0,0,0])

# Pendant pos: 261.8, -46.06, 96.02, -138.23, -94.13, 35.86
# Gazebo pos : 85.56 -46.91 102.72 -57.6 -88.45 35.74
# diff        180    0      0       -90       0      0

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

"""
Define a ROS topic callback funtion for getting the state of suction cup
Whenever ur3/gripper_input publishes info this callback function is called.
"""

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
    ja_loc = lab_invk(loc[0], loc[1], loc[2], 0)
    ja_loc_up = lab_invk(loc[0], loc[1], loc[2]+0.2, 0)
    ja_end_up = lab_invk(end[0], end[1], end[2]+0.2, 0)
    ja_end = lab_invk(end[0], end[1], end[2], 0)

    rospy.loginfo("Moving to: " + str(loc))

    move_arm(pub_cmd, loop_rate, ja_loc_up, 4.0, 4.0)
    move_arm(pub_cmd, loop_rate, ja_loc, 4.0, 4.0)

    rospy.loginfo("Gripper on")
    gripper(pub_cmd, loop_rate, suction_on)
    time.sleep(0.5)

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
    #
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
            #rospy.loginfo("Goal is reached!")

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error



def main():

    global home
    global Q
    global SPIN_RATE

    # Initialize ROS node
    rospy.init_node('lab2node')

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

    # time.sleep(2)

    #define a ROS publisher to move the cart
    cart_move = rospy.Publisher('cart/cmd_vel', Twist, queue_size=10)

    # time.sleep(5)

    # Initialize subscriber to ur3/position and callback fuction
    # each time data is published
    sub_position = rospy.Subscriber('ur3/position', position, position_callback)

    # define a ROS subscriber for ur3/gripper_input message and corresponding callback function
    sub_gripper = rospy.Subscriber('ur3/gripper_input', gripper_input, gripper_callback)


    # Check if ROS is ready for operation
    while(rospy.is_shutdown()):
        print("ROS is shutdown!")

    rospy.loginfo("Sending Goals ...")
    loop_rate = rospy.Rate(SPIN_RATE)

    # move = Twist()
    # move.linear.x = 1
    # move.angular.z = 0
    # cart_move.publish(move)

    pos_box = [0.3, 0.1, -0.155, 0]
    pos_end = [0.3, -0.1, -0.155, 0]
    status = move_block(pub_command, loop_rate, pos_box, pos_end)
    # status = move_block(pub_command, loop_rate, Q2)
    
if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass
