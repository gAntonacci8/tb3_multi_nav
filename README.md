Tb3 multi nav tag-game \

PROTO_SCRATCH BRANCH -- Testing creation of Nav2 launcher and nodes upbringing from scratch. 

STATUS: NOT WORKING \
- Cartographer ONLINE, saves map. 
- Navigation2 Stack PARTIALLY ONLINE, works but not correctly.
- Robot accepts pose and goals from Rviz. Navigation shows signs of not adherence to track. 
- Map saving and loading tested: saving .yaml and .pgm correctly, loads and publish to /map correctly.
- Rviz accepts map and map frame. Problems for reloading map frame when opening premade map.
- Autonomous navigation: NOT WORKING 
- !! Multi-Robot: NOT TESTED YET.

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
Mapping an uknown map for .yaml and .pgm map file to Rviz2 \
Gazebo <--> ROS <--> Rviz2 integration \
Rviz2 multi robot configuration \
Data categorization with namespaces \
Bringup setup for multi robot

