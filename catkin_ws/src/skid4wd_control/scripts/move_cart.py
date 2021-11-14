#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

rospy.init_node("cartnode")

movement = Twist()
movement.linear.x = 0.2
# movement.linear.y = -10

movement.angular.z = 0.5

cart_move = rospy.Publisher('/drive_controller/cmd_vel',Twist,queue_size=1)

while True:
    cart_move.publish(movement)
