#!/usr/bin/env python
import sys, rospy, tf, rospkg
from gazebo_msgs.srv import *
from geometry_msgs.msg import *
from copy import deepcopy
from pick_and_place_moveit import PickAndPlaceMoveIt

from std_msgs.msg import (
    Empty,
)

from geometry_msgs.msg import (
    Pose,
    Point,
    Quaternion,
)

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)

import baxter_interface
import moveit_commander

if __name__ == '__main__':
    # initialize pick and place
    # get starting pose from spawn file
    # get goal pose from spawn file
    pass

def move_piece(spawn_pose, ending_pose,overhead_orientation):
    moveit_commander.roscpp_initialize(sys.argv)
    # rospy.init_node("ik_pick_and_place_moveit")

    # Wait for the All Clear from emulator startup
    rospy.wait_for_message("/robot/sim/started", Empty)

    limb = 'left'
    hover_distance = 0.15  # meters

    # An orientation for gripper fingers to be overhead and parallel to the obj
    overhead_orientation = Quaternion(x=-0.0249590815779, y=0.999649402929, z=0.00737916180073, w=0.00486450832011)
    # NOTE: Gazebo and Rviz has different origins, even though they are connected. For this
    # we need to compensate for this offset which is 0.93 from the ground in gazebo to
    # the actual 0, 0, 0 in Rviz.
    central_pose = Pose(
        position= Point(x=0.54275, y=0.30625, z=0.798 + hover_distance),
        orientation= overhead_orientation
    )
    pnp = PickAndPlaceMoveIt(limb, hover_distance)

    # Move to the desired starting angles
    pnp.move_to_start(central_pose)
    pnp.pick(spawn_pose)
    pnp._approach(central_pose)
    pnp.place(ending_pose)
    pnp._approach(central_pose)    
    del pnp

def go_to_start():
    moveit_commander.roscpp_initialize(sys.argv)
    # rospy.init_node("ik_pick_and_place_moveit")

    # Wait for the All Clear from emulator startup
    rospy.wait_for_message("/robot/sim/started", Empty)

    limb = 'left'
    hover_distance = 0.15  # meters
    overhead_orientation = Quaternion(x=-0.0249590815779, y=0.999649402929, z=0.00737916180073, w=0.00486450832011)
    # NOTE: Gazebo and Rviz has different origins, even though they are connected. For this
    # we need to compensate for this offset which is 0.93 from the ground in gazebo to
    # the actual 0, 0, 0 in Rviz.
    central_pose = Pose(
        position= Point(x=0.54275, y=0.30625, z=0.798 + hover_distance),
        orientation= overhead_orientation
    )
    pnp = PickAndPlaceMoveIt(limb, hover_distance)
    pnp.move_to_start(central_pose)
    del pnp
