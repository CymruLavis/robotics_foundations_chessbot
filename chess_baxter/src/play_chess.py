#!/usr/bin/env python
import sys, rospy, tf, rospkg
from gazebo_msgs.srv import *
from geometry_msgs.msg import *
from copy import deepcopy

if __name__ == '__main__':
    
    # sequence of chess moves
    chess_moves = ['move1', 'move2', 'move3', 'move4']
    move_mechanics = {
        "move1": {
            "pose_x":1,
            "pose_y":1,
            "pose_z":1,
            "pose_w":1,
        },
    }
    # initialize pick and place node
    
    pass