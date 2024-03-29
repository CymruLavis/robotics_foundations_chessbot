#!/usr/bin/env python
import sys, rospy, tf, rospkg
from gazebo_msgs.srv import *
from geometry_msgs.msg import *
from copy import deepcopy
import setup_chess_board
from std_msgs.msg import (
    Empty,
)
from setup_chess_board import PickAndPlaceMoveIt


# http://wiki.ros.org/simulator_gazebo/Tutorials/ListOfMaterials

if __name__ == '__main__':
    rospy.init_node("spawn_chessboard")
    rospy.wait_for_service("gazebo/spawn_sdf_model")

    
    srv_call = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    overhead_orientation = Quaternion(x=-0.0249590815779, y=0.999649402929, z=0.00737916180073, w=0.00486450832011)
    hover_distance = 0.15  # meters
    limb = 'left'
    starting_pose = Pose(position=Point(x=0.5, y=0.4, z=0.0), orientation=overhead_orientation) #starting pose may need to change due to rviz origin difference
    
    pnp = PickAndPlaceMoveIt(limb, hover_distance=hover_distance)
    pnp.move_to_start(starting_pose)

    # Table
    model_path = rospkg.RosPack().get_path('baxter_sim_examples')+"/models/"
    table_xml = ''
    with open(model_path + "cafe_table/model.sdf", "r") as table_file:
        table_xml = table_file.read().replace('\n', '')

    table_pose=Pose(position=Point(x=0.73, y=0.4, z=0.0))
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        spawn_sdf("cafe_table", table_xml, "/", table_pose, "world")
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))
    

    # ChessBoard
    orient = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0))
    board_pose = Pose(Point(0.3,0.55,0.78), orient)
    frame_dist = 0.025
    model_path = rospkg.RosPack().get_path('chess_baxter')+"/models/"
    with open(model_path + "chessboard/model.sdf", "r") as f:
        board_xml = f.read().replace('\n', '')

    # print("board xml: \n " + board_xml + "\n\n\n")
    # Add chessboard into the simulation
    print srv_call("chessboard", board_xml, "", board_pose, "world")

    # Add chesspieces into the simulation
    origin_piece = 0.03125

    pieces_xml = dict()
    list_pieces = 'rnbqkpRNBQKP'
    for each in list_pieces:
        with open(model_path + each+".sdf", "r") as f:
            pieces_xml[each] = f.read().replace('\n', '')

    # board_setup = ['rnbqkbnr', 'pppppppp', '********', '********', '********', '********', 'PPPPPPPP', 'RNBQKBNR']
    # board_setup = ['r******r', '', '**k*****', '', '', '******K*', '', 'R******R']
    board_setup = ['r*bqk*nr', '***pp***', '********', '********', '********', '******K*', '********', 'R******R']

    piece_positionmap = dict()
    piece_names = []
    grid = []

    # # Initialize pick and place 
    
    common_pose = Pose(Point(0.57,0.3,0.8+hover_distance), overhead_orientation)
    spawn_pose = Pose(Point(0.57, 0.3, 0.8), orient)
    for row, each in enumerate(board_setup):
        # print row
        for col, piece in enumerate(each):
            pose = deepcopy(board_pose)
            pose.position.x = board_pose.position.x + frame_dist + origin_piece + row * (2 * origin_piece)
            pose.position.y = board_pose.position.y - 0.55 + frame_dist + origin_piece + col * (2 * origin_piece)
            pose.position.z += 0.018
            piece_positionmap[str(row)+str(col)] = [pose.position.x, pose.position.y, pose.position.z-0.93] #0.93 to compensate Gazebo RViz origin difference
            grid.append([pose.position.x, pose.position.y, pose.position.z-0.93])
            if piece in list_pieces:
                piece_names.append("%s%d" % (piece,col))
                piece_name = piece+str(col)
                try:
                    spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
                    spawn_sdf(piece_name, pieces_xml[piece], "/", spawn_pose, "world")
                    # spawn_sdf(piece_name, pieces_xml[piece], "/", pose, "world")

                    # THIS WORKS, JUST UNCOMMENT
                    pick = Pose(position=Point(x=spawn_pose.position.x, y= spawn_pose.position.y, z=spawn_pose.position.z-0.93),
                        orientation= overhead_orientation)

                    place = Pose(position=Point(x=pose.position.x, y=pose.position.y, z=pose.position.z-0.93),
                            orientation= overhead_orientation)
                    
                    pnp.pick(pick)
                    pnp.move_to_common_point(common_pose)
                    release_pose = Pose(Point(x= pose.position.x, y=pose.position.y, z=pose.position.z-0.93+hover_distance), overhead_orientation)
                    pnp.move_to_common_point(release_pose)
                    pnp.place(place)
                    pnp.move_to_common_point(release_pose)
                    pnp.move_to_common_point(common_pose)

                except rospy.ServiceException, e:
                    rospy.logerr("Spawn SDF service call failed: {0}".format(e))
            

                
    rospy.set_param('grid_poses', grid)
    rospy.set_param('board_setup', board_setup) # Board setup
    rospy.set_param('list_pieces', list_pieces) # List of unique pieces
    rospy.set_param('piece_target_position_map', piece_positionmap) # 3D positions for each square in the chessboard
    rospy.set_param('piece_names', piece_names) # Pieces that will be part of the game
    rospy.set_param('pieces_xml', pieces_xml) # File paths to Gazebo models, i.e. SDF files
    # setup_chess_board.go_to_start()

