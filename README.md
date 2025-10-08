Tb3 multi nav tag-game \

JAZZY_ROBOT1_NAMESPACED BRANCH -- Single robot simulation, with namespace "robot1". New branch created as it differs too much from other version. 

For not namespaced single robot simulation, refer to <proto_scratch> branch

This branch will be left as a single robot operational simulation, once completed. \
Multi-robot branch TBD.

STATUS: SINGLE ROBOT SHOWING, WORKING WITH KEYBOARD CONTROLS (remapped) AND NAV2 PLANNING. 

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
ROS2 Distro - Jazzy \
Gazebo Version - Harmonic \
Rviz2 

External Repositories (used partially or entirely):

- turtlebot3_gazebo (ROBOTIS): https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/jazzy 
- turtlebot3-jazzy  (ROBOTIS): https://github.com/ROBOTIS-GIT/turtlebot3/tree/jazzy 


Currently working on:

Modular launcher for multi robot nav2 stack \
Mapping an uknown map for .yaml and .pgm map file to Rviz2 (DONE) \
Gazebo <--> ROS <--> Rviz2 integration \
Rviz2 multi robot configuration \
Data categorization with namespaces \
Bringup setup for multi robot

