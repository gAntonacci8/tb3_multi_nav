# Tb3 multi nav tag-game

MAIN BRANCH -- Multi robot simulation, with namespaces "robot1","robot2". 

University project utilizing 2 turtlebot3 waffle robots for a simulated tag game. \
Using Nav2 stack, Gazebo bridged to ROS2 and Rviz for sensor monitoring. 

### YouTube Video
[![ Tag Game In Gazebo Harmonic: Orchestrated Dual-Robot Navigation ](https://img.youtube.com/vi/A-RKV6gecJI/maxresdefault.jpg)](https://www.youtube.com/watch?v=A-RKV6gecJI)


>**Note: This branch is the only one intended to be used for the simulation.** 

## ================ REPRODUCIBILITY STEPS ================

To reproduce the setup:
1. Make sure that Docker is active and running.

1.	Open Windows Terminal and start the Linux environment with command
    ```bash
    wsl
    ```
1.	If WSL2 with Ubuntu 22.04 is not installed, it can be set up by following the official [Microsoft documentation for WSL2 installation](https://learn.microsoft.com/en-us/windows/wsl/install). \
Clone the project repository inside the Ubuntu shell:
    ```bash
    cd ~
    git clone -b main https://github.com/gAntonacci8/tb3_multi_nav.git
    cd tb3_multi_nav
    ```
1.  The simulation runs inside a Docker container that already includes ROS 2 Jazzy, Gazebo Harmonic and the required dependencies.
    From inside the project folder (in Ubuntu/WSL):
    ```bash
    cd docker_ws
    ./builder.sh
    ```
    The builder.sh script builds the appropriate Docker image from Docker Hub and prepares the environment needed to run the project.

    After building the image, go back to the main folder:
    ```bash
    cd ..
    ```
    To start the container, one of the following scripts can be used:
    ```bash
    ./run.sh       
    ```
    or
    ```bash
    ./gpu_run.sh
    ``` 
    > **Note: when first entering the container, two sourcing *"No such file or directory"* messages will appear. This is normal and will disappear when compiling the package using the next command.** 

    The script ***run.sh*** is used for CPU-only execution while the script ***./gpu_run.sh*** is used for execution with NVIDIA GPUs support, typically available on standard laptops/desktops.
    Once inside the running container:
    ```bash
    cd /root/ros_ws
    source ./compile.sh
    ``` 
1. From inside the container and after building, run the following script:
    ```bash
    ./sim_start.sh 
    ```
## =================== DATA OUTPUT ===================
All rounds' data are written under: 
```bash
~/ros_ws/tag_metrics
```
For each experiment, the system produces: 
 - episode_XXX.csv – detailed, time-stamped metrics per round, 
 - all_runs.csv – one aggregated row per round (outcome, winner, scores, tag/safe counts, distances, remaining paths, etc.).
## ================== AUXILIARY SCRIPTS ==================
- Assigning the project directory to the current user can be perfomed by running the following script:
    ```bash
     ./chown_me.sh 
    ```

- To enter the running container from another terminal, execute the following script:
    ```bash
     ./exec.sh 
    ```

- In addition to **sim_start.sh**, two alternative launch scripts are available for debugging and log persistence:
    ```bash
     ./file_sim_start.sh 
    ```
    Launches the simulation and redirects all console output to persistent log files, preventing loss of diagnostic information.
    ```bash
    ./verbose_sim_start.sh 
    ```
    Performs the same file-based logging but with ROS 2 debug output enabled, offering maximum verbosity for detailed troubleshooting.

- For TF debugging, the project also provides:
    ```bash
     ./checkframes.sh 
    ```
    It generates a complete TF tree snapshot by invoking *tf2_tools view_frames*, saving the resulting diagram as a PDF inside the **frames_view/** directory.

- To opens project's ***setup.py*** file for quick editing:
    ```bash
    ./editconfig.sh
    ```
- To open ***launch/*** folder and edit the launch file:
    ```bash
    ./gotolaunch.sh
    ```
- To open the inner ***tb3_multi_nav/*** package and edit the .xml package file:
    ```bash
    ./gotopackage.sh
    ```
- To manually control robot1 using the keyboard:
    > **IMPORTANT: It might disrupt current simulation.**
    ```bash
    ./keyboard1.sh
    ```

##  ================== ADDITIONAL INFO ==================

For not namespaced single robot simulation, refer to [proto_scratch](https://github.com/gAntonacci8/tb3_multi_nav/tree/proto_scratch) branch. Beware that parts of this branch might be outdated  compared the others.

For single robot namespaced simulation, refer to [jazzy_robot1_namespaced](https://github.com/gAntonacci8/tb3_multi_nav/tree/jazzy_robot1_namespaced) branch. Said branch will be "cloned" and adapted for multi-robot purposes.

Config/ file namespaced to "robotX" where needed. 

**STATUS**: **FINISHED**. BASIC BRINGUP ONLINE, NAV2 ONLINE. TAG GAME IMPLEMENTED. 

- Navigation2 Stack  ONLINE and responsive.
- Bt_navigator XML files customized with namespaces for both robots.
- Robot1 can be controlled by teleop keyboard using the script **"keyboard1.sh"**, remapping /cmd_vel to /robot1/cmd_vel. Both robots controllable by Rviz or by publishing goal_pose to specific namespaced topic.
- Map behavior tested: loads and publishes to /map correctly.
- Rviz accepts map and map frame. Correct behavior when reloading custom map from map_server. Initial pose by Rviz accepted correctly and set for both robots.
- Rviz recognizes Nav2 servers if it's launched with <namespace: "robot1"> and <namespace: "robot2">.
- Autonomous navigation: Nav2goal navigates robots to goal pose.



Setup: 
- ROS2 Distro - Jazzy 
- Gazebo Version - Harmonic, v. 8.9.0
- Rviz2, v. 14.1.14 (Compiled against QT version 5.15.13, OGRE version 1.12.10)

External Repositories (used partially or entirely):

- turtlebot3_gazebo (ROBOTIS): https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/jazzy 
- turtlebot3-jazzy  (ROBOTIS): https://github.com/ROBOTIS-GIT/turtlebot3/tree/jazzy 


Roadmap:

- Modular launcher for multi robot nav2 stack (DONE) 
- Mapping an uknown map for .yaml and .pgm map file to Rviz2 (DONE) 
- Gazebo <--> ROS <--> Rviz2 integration (DONE) 
- Rviz2 multi robot configuration (DONE WITH 2 RVIZ INSTANCES) 
- Data categorization with namespaces (DONE) 
- Bringup setup for multi robot (DONE)
