'''
Main launcher for multi robot tag-game (2 robots). 

Namespaces: robot1, robot2

Tentativa approach to modular navigation2 nodes, manually configuring namespaces.
'''
from launch import LaunchDescription
from launch.actions import ExecuteProcess,TimerAction, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LifecycleNode,Node, PushRosNamespace
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    namespaces=['robot1','robot2']

    #------------ Open Gazebo and spawn two (2) turtlebot3 waffle
    #--- Coordinates: Robot1 (-0.5 0 0.01 0 0 0), Robot2 (-0.5 -1 0 0 0 0.01) 

    tb3_world = os.path.join(                               #get turtlebot3 world for gazebo
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds', 'turtlebot3_world.world'
    )
    model_folder="turtlebot3_waffle"                        
    sdf_path = os.path.join(                                #sdf path to waffle robot file
    get_package_share_directory('turtlebot3_gazebo'),
    'models',
    model_folder,
    'model.sdf'
    )
    #Launch Gazebo --- SIMPLE COMMAND OR NODE? --> Node for now
    gazebo = IncludeLaunchDescription(                     
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': f'-r {tb3_world}'}.items() #maybe 'use_sim_time':'true' ?
    )

