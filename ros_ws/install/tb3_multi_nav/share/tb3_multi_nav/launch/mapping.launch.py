'''
Tentative approach to create a separate launcher which maps a gazebo world into .yaml and possibly a .pgm
for Rviz.

After mapping, these launch will not be used, as multi_tb3.launch.py collects main logic.
'''


from launch import LaunchDescription
from launch.actions import ExecuteProcess,TimerAction, IncludeLaunchDescription, GroupAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LifecycleNode,Node, PushRosNamespace
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    explore_lite=Node(
        package=""

    )

    return LaunchDescription([


    ])