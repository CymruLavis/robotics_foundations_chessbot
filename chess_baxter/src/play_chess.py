#!/usr/bin/env python
import sys, rospy, tf, rospkg
from gazebo_msgs.srv import *
from geometry_msgs.msg import *
from copy import deepcopy
from geometry_msgs.msg import (
    Pose,
    Point,
    Quaternion,
)
import gazebo2tfframe
import setup_chess_board
#!/usr/bin/env python
import rospy
import tf
from gazebo_msgs.msg import LinkStates

# This is hard-coded to block for this exercise, yet you can make the script general by adding cmd line arguments

# Global variable where the object's pose is stored
pose = None


def get_links_gazebo(link_states_msg):
    # Call back to retrieve the object you are interested in
    global input_linkname
    global pose
    poses = {'world': link_states_msg.pose[0]} # get world link
    for (link_idx, link_name) in enumerate(link_states_msg.name):
        modelname = link_name.split('::')[0]
        if input_linkname == modelname:
            poses[modelname] = link_states_msg.pose[link_idx]

    pose = poses[input_linkname]


def get_piece_pose(input_linkname):
    rospy.init_node('gazebo2tfframe')

    # Create TF broadcaster -- this will publish a frame give a pose
    tfBroadcaster = tf.TransformBroadcaster()
    # SUbscribe to Gazebo's topic where all links and objects poses within the simulation are published
    rospy.Subscriber('gazebo/link_states', LinkStates, get_links_gazebo)

    rospy.loginfo('Spinning')
    global pose
    rate = rospy.Rate(20)
    # while not rospy.is_shutdown():
    if pose is not None:
        pos = pose.position
        ori = pose.orientation
        rospy.loginfo(pos)
        # Publish transformation given in pose
        tfBroadcaster.sendTransform((pos.x, pos.y, pos.z - 0.93), (ori.x, ori.y, ori.z, ori.w), rospy.Time.now(), input_linkname, 'world')
        # rate.sleep()
    return pos
    # rospy.spin()


# if __name__ == '__main__':
#     main()
if __name__ == '__main__':
    row = 0.2
    col = 0.2
    # sequence of chess moves
    chess_pieces = [['r0', 'n1', 'b2', 'q3', 'k4', 'b5', 'n6', 'r7'],
                    ['p1', 'p2', 'p3', 'p4', 'p5', 'p6', 'p7'],
                    ['P1', 'P2', 'P3', 'P4', 'P5', 'P6', 'P7'],
                   [ 'R0', 'N1', 'B2', 'Q3', 'K4', 'B5', 'N6', 'R7']]

    chess_moves = ['move1', 'move2', 'move3', 'move4', 'move5', 'move6']
    move_pieces = ['p4', 'p3', 'n6', 'b2', 'q3', 'k4']
    move_mechanics = {}
    overhead_orientation = Quaternion(x=-0.0249590815779, y=0.999649402929, z=0.00737916180073, w=0.00486450832011)
    commonPose = Pose(
    position= Point(x=0.54275, y=0.30625, z=0.798 + hover_distance),
    orientation= overhead_orientation
    )    
    

    for i in range(len(chess_moves)):
        pos = get_piece_pose(move_pieces[i])
        move_mechanics[chess_moves[i]] = {
            "StartingPose": Pose(
                position= Point(x=pos.x, y=pos.y, z=pos.z-0.93 + hover_distance),
                orientation= overhead_orientation
            )
        }
    # initialize pick and place node
    move_mechanics['move1']['EndingPose'] = Pose(
        position= Point(x=move_mechanics['move1']['EndingPose'].position.x, y=move_mechanics['move1']['EndingPose'].position.y+(row*2), z=move_mechanics['move1']['EndingPose'].position.z ),
        orientaiton= overhead_orientation
    )
    move_mechanics['move2']['EndingPose'] = Pose()
    move_mechanics['move3']['EndingPose'] = Pose()
    move_mechanics['move4']['EndingPose'] = Pose()
    move_mechanics['move5']['EndingPose'] = Pose()
    move_mechanics['move6']['EndingPose'] = Pose()

    for key, value in move_mechanics:
        setup_chess_board.move_piece(value['StartingPose'], value['EndingPose'],overhead_orientation)