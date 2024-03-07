#!/usr/bin/env python
import sys
import copy

import rospy
import rospkg

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
from setup_chess_board import PickAndPlaceMoveIt
from gazebo2tfframe import get_saved_poses
overhead_orientation = Quaternion(x=-0.0249590815779, y=0.999649402929, z=0.00737916180073, w=0.00486450832011)

def movement_setup():
    # sequence of chess moves
    pos_map = rospy.get_param("piece_target_position_map")
    full_map = rospy.get_param('grid_poses')
   
    # print(type(pos_map))
    # print(pos_map)
    # pos_map = [ [ 0, 1, 2, 3, 4, 5, 6, 7],
    #             [ 8, 9,10,11,12,13,14,15],
    #             [16,17,18,19,20,21,22,23],
    #             [24,25,26,27,28,29,30,31],
    #             [32,33,34,35,36,37,38,39],
    #             [40,41,42,43,44,45,46,47],
    #             [48,49,50,51,52,53,54,55],
    #             [56,57,58,59,60,61,62,63]]
    # chess_pieces = [['r0', 'n1', 'b2', 'q3', 'k4', 'b5', 'n6', 'r7'],
    #                 ['p0', 'p1', 'p2', 'p3', 'p4', 'p5', 'p6', 'p7'],
    #                 ['P0', 'P1', 'P2', 'P3', 'P4', 'P5', 'P6', 'P7'],
    #                 ['R0', 'N1', 'B2', 'Q3', 'K4', 'B5', 'N6', 'R7']]

    pick_pieces = ['p4', 'p3', 'n6', 'b2', 'q3', 'k4']
    pick_poses = []
    places = [28,19,23,38,11,3]
    place_poses = []
    
    returned_poses = get_saved_poses()
    print("Returned Poses")
    print(returned_poses)
    # for piece in pick_pieces:
        # pose = gazebo2tfframe.get_pose(piece)
        # print(pose)
        # pick_poses.append(Pose(position=Point(x=pose[0], y=pose[1], z=pose[2]), orientation=overhead_orientation))

    for place in places:
        pose = full_map[place]
        # print(pose)
        place_poses.append(Pose(position=Point(x=pose[0], y=pose[1], z=pose[2]), orientation= overhead_orientation))

    return pick_poses, place_poses

def main():
    # moveit_commander.roscpp_initialize(sys.argv)
    # rospy.init_node("ik_pick_and_place_moveit")
    # rospy.wait_for_message("/robot/sim/started", Empty)

    limb = 'left'
    hover_distance = 0.15  # meters
    # overhead_orientation = Quaternion(x=-0.0249590815779, y=0.999649402929, z=0.00737916180073, w=0.00486450832011)
    common_pose = Pose(Point(0.57,0.3,0.8+hover_distance), overhead_orientation)
    
    # pnp = PickAndPlaceMoveIt(limb, hover_distance)   
    # pnp.move_to_start(common_pose)
    picks, places = movement_setup()
    

    # pnp.pick(pick)
    # pnp.move_to_common_point(common_pose)
    # release_pose = Pose(Point(x= pose.position.x, y=pose.position.y, z=pose.position.z-0.93+hover_distance), overhead_orientation)
    # pnp.move_to_common_point(release_pose)
    # pnp.place(place)
    # pnp.move_to_common_point(release_pose)
    # pnp.move_to_common_point(common_pose)

    for i in range(len(picks)):
        print("PICK POSE:\n", picks[i])
        print("PLACE POSE:\n", places[i])
        
        # pnp.pick(picks[i])

        # pnp.place([places[i]])


    return 0


if __name__ == '__main__':
    sys.exit(main())

