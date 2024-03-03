This purpose of this project was to utilized provided PickAndPlace class, gazebo2tfframe package, and baxter sdk to create a simulation of the baxter robot playing chess.
The simulation was done using Gazebo and Rviz and all communication was done using ROS
With all appropriate SDK's an packages downloaded, the following commands will execute the prcoess

Terminal 1:
  roslaunch baxter_gazebo baxter_world.launch

Terminal 2:
  rosrun baxter_tools enable_robot.py -e
  rosrun baxter_interface joint_trajectory_action_server.py

Terminal 3:
  roslaunch baxter_moveit_config baxter_grippers.launch

Terminal 4:
  rosrun chess_baxter spawn_chessboard_h.py

Terminal 5: --> this one may be unnesecarry
  rosrun chess_baxter gazebo2tfframe.py

Terminal 6:
  rosrun chess_baxter play_chess.py
