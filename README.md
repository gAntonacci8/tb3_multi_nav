Tb3 multi nav tag-game \

PROTO_MULTI_ROBOT BRANCH -- Multi robot simulation, with namespaces "robot1","robot2".

Planning tag-game with 2 robots. 

For not namespaced single robot simulation, refer to <proto_scratch> branch

For single robot namespaced simulation, refer to <jazzy_robot1_namespaced> branch. Said branch will be "cloned" and adapted for multi-robot purposes.

Config/ file namespaced to "robotX" where needed. 

STATUS: PROTOTYPE, BASIC BRINGUP ONLINE, NAV2 ONLINE. 

- Navigation2 Stack  ONLINE and responsive.
- Bt_navigator XML fileS customized with namespaces for both robots.
- Robot1 can be controlled by teleop keyboard, remapping /cmd_vel to /robot1/cmd_vel. Both robots controllable by Rviz or by publishing goal_pose to specific namespaced topic.
- Map saving and loading tested: saving .yaml and .pgm correctly, loads and publish to /map correctly.
- Rviz accepts map and map frame. Correct behavior when reloading custom map from map_server. Initial pose by Rviz accepted correctly and set for both robots.
- Rviz recognizes Nav2 servers if it's launched with <namespace: "robot1"> and <namespace: "robot2">.
- Autonomous navigation: Nav2goal navigates robots to goal pose. Some issues in rotation, light fix applied, still working on it.
- Navigation and Localization in Rviz2 Navigation Plugin still "Unknown". No fix found atm.

University project utilizing 2 turtlebot3 waffle robots for a simulated tag game. \
Using Nav2 stack, Gazebo bridged to ROS2 and Rviz for sensor monitoring.

Setup: 
- ROS2 Distro - Jazzy 
- Gazebo Version - Harmonic, v. 8.9.0
- Rviz2, v. 14.1.14 (Compiled againt QT version 5.15.13, OGRE version 1.12.10)

External Repositories (used partially or entirely):

- turtlebot3_gazebo (ROBOTIS): https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/jazzy 
- turtlebot3-jazzy  (ROBOTIS): https://github.com/ROBOTIS-GIT/turtlebot3/tree/jazzy 


Currently working on:

Modular launcher for multi robot nav2 stack (BASIC BRINGUP DONE) \
Mapping an uknown map for .yaml and .pgm map file to Rviz2 (DONE) \
Gazebo <--> ROS <--> Rviz2 integration (OK FOR BASIC SIM) \
Rviz2 multi robot configuration (DONE WITH 2 RVIZ INSTANCES) \
Data categorization with namespaces (DONE) \
Bringup setup for multi robot (DONE)

