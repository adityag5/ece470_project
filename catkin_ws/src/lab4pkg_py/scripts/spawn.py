#!/usr/bin/env python

import rospy
import rospkg
import os
import sys
import yaml
from random import random
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import DeleteModel
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

def rand_pos(dist):
    return random() * dist - (0.5) * dist

# Initialize rospack
rospack = rospkg.RosPack()

# Initialize ROS node
rospy.init_node('ur3_gazebo_spawner', anonymous=True)
# Initialize ROS pack
rospack = rospkg.RosPack()

# Get path to block
ur_path = rospack.get_path('ur_description')
block1_path = os.path.join(ur_path, 'urdf', 'block_green.urdf')
block2_path = os.path.join(ur_path, 'urdf', 'block_red.urdf')
block3_path = os.path.join(ur_path, 'urdf', 'block_green.urdf')

# Wait for service to start
rospy.wait_for_service('gazebo/spawn_urdf_model')
spawn = rospy.ServiceProxy('gazebo/spawn_urdf_model', SpawnModel)
delete = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)

name1, name2, name3 = "block1", "block2", "block3"

diameter = 1
# pose1 = Pose(Point(0.3, 0.1, .1), Quaternion(0, 0, 0, 0))
# print("POSE 1: (", str(pose1.position.x), ",", str(pose1.position.y), ")")

pose2 = Pose(Point(0.3, 0.1, 0.017), Quaternion(0, 0, 0, 0))
print("POSE 2: (", str(pose2.position.x), ",", str(pose2.position.y), ")")

#pose3 = Pose(Point(rand_pos(diameter), rand_pos(diameter), 1), Quaternion(0, 0, 0, 0))
#print("POSE 3: (", str(pose3.position.x), ",", str(pose3.position.y), ")")


# spawn(name1, open(block1_path, 'r').read(), 'block', pose1, 'world')
spawn(name2, open(block2_path, 'r').read(), 'block', pose2, 'world')
#spawn(name3, open(block3_path, 'r').read(), 'block', pose3, 'world')



