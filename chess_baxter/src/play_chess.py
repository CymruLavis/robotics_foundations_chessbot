#!/usr/bin/env python
import sys
import copy

import rospy
import rospkg
import tf

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

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState

def get_pose(model_name):
    rospy.wait_for_service('/gazebo/get_model_state')
    model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    response = model_state(model_name, '')
    response.pose.position.z = response.pose.position.z-0.93
    return response.pose

def movement_setup():
    # sequence of chess moves
    overhead_orientation = Quaternion(x=-0.025, y=1, z=0.007, w=0.005)

    pos_map = rospy.get_param("piece_target_position_map")
    full_map = rospy.get_param('grid_poses')
   
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
    # rospy.init_node('get_model_pose_node')
    
    for piece in pick_pieces:
        pose = get_pose(piece)
        pick_poses.append(Pose(position=Point(x=pose.position.x, y=pose.position.y, z=pose.position.z), orientation=overhead_orientation))
    
    for place in places:
        print(len(full_map))
        pose = full_map[place]
        place_poses.append(Pose(position=Point(x=pose[0], y=pose[1], z=pose[2]), orientation= overhead_orientation))

    return pick_poses, place_poses

def main():
    
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("ik_pick_and_place_moveit")
    rospy.wait_for_message("/robot/sim/started", Empty)
    overhead_orientation = Quaternion(x=-0.025, y=1, z=0.007, w=0.005)

    limb = 'left'
    hover_distance = 0.15  # meters
    # overhead_orientation = Quaternion(x=-0.0249590815779, y=0.999649402929, z=0.00737916180073, w=0.00486450832011)
    common_pose = Pose(Point(0.57,0.3,0.8+hover_distance), overhead_orientation)
    
    pnp = PickAndPlaceMoveIt(limb, hover_distance)   
    pnp.move_to_start(common_pose)
    picks, places = movement_setup()


    for i in range(len(picks)):
        # print("PICK POSE FOR:\n", picks[i])
        # print("PLACE POSE:\n", places[i])
        # this_pick = picks[i]
        # this_place = places[i]

        over_pick_pose = Pose(Point(x=picks[i].position.x, y=picks[i].position.y, z=picks[i].position.z-0.93+hover_distance), overhead_orientation)
        over_place_pose = Pose(Point(x= places[i].position.x, y=places[i].position.y, z=places[i].position.z-0.93+hover_distance), overhead_orientation)

        # hover over pick piece, pick it up, hover over pick piece
        pnp.move_to_common_point(over_pick_pose)
        pnp.pick(picks[i])
        pnp.move_to_common_point(over_pick_pose)

        # hover over place location, place, hover over place location
        pnp.move_to_common_point(over_place_pose)
        pnp.place(places[i])
        pnp.move_to_common_point(over_place_pose)

       
    pnp.move_to_start(common_pose)
    return 0


if __name__ == '__main__':
    sys.exit(main())
    

