Tb3 multi nav tag-game \

PROTO_MULTI_ROBOT BRANCH -- Multi robot simulation, with namespaces "robot1","robot2".

Planning tag-game with 2 robots. 

For not namespaced single robot simulation, refer to <proto_scratch> branch

For single robot namespaced simulation, refer to <jazzy_robot1_namespaced> branch. Said branch will be "cloned" and adapted for multi-robot purposes.

Config/ file namespaced to "robotX" where needed. 

STATUS: EARLY PROTOTYPE. 

[LIST NOT RELIABLE. LEFT AS COMPARISON TO ]
- Navigation2 Stack  ONLINE and responsive.
- Bt_navigator XML file customized with namespace. Evaluating xacro like sintax to dynamically include namespace where needed.
- Robot can be controlled by teleop keyboard, remapping /cmd_vel to /robot1/cmd_vel. Other controllers not working atm.
- Map saving and loading tested: saving .yaml and .pgm correctly, loads and publish to /map correctly.
- Rviz accepts map and map frame. Correct behavior when reloading custom map from map_server. Initial pose by Rviz accepted correctly.
- Rviz recognizes Nav2 servers if it's launched with <namespace: "robot1">.
- Autonomous navigation: Nav2goal navigates robot to goal pose. Some issues in rotation, no fix on the agenda atm.
- Navigation and Localization in Rviz2 Navigation Plugin still "Unknown".
- !! Multi-Robot: NOT TESTED YET. Will be tested in a separate branch

University project utilizing 2 turtlebot3 waffle robots for a simulated tag game. \
Using Nav2 stack, Gazebo bridged to ROS2 and Rviz for sensor monitoring.

Setup: \
- ROS2 Distro - Jazzy 
- Gazebo Version - Harmonic 
- Rviz2 

External Repositories (used partially or entirely):

- turtlebot3_gazebo (ROBOTIS): https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/jazzy 
- turtlebot3-jazzy  (ROBOTIS): https://github.com/ROBOTIS-GIT/turtlebot3/tree/jazzy 


Currently working on:

Modular launcher for multi robot nav2 stack (SINGLE BOT MODULAR NODE LAUNCHER DONE) \
Mapping an uknown map for .yaml and .pgm map file to Rviz2 (DONE) \
Gazebo <--> ROS <--> Rviz2 integration (SINGLE BOT OK FAR) \
Rviz2 multi robot configuration \
Data categorization with namespaces (SINGLE BOT WORKING) \
Bringup setup for multi robot 

