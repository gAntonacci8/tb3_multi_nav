# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    # Get the urdf file
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL
    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        model_folder,
        'model.sdf'
    )

    # Launch configuration variables specific to simulation
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    x_pose_1 = LaunchConfiguration('x_pose_1', default='0.5')
    y_pose_1 = LaunchConfiguration('y_pose_1', default='0.5')
    
    # Declare the launch arguments
    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='Specify namespace of the robot')

    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Specify namespace of the robot')

    declare_x_1_position_cmd = DeclareLaunchArgument(
        'x_pose_1', default_value='0.5',
        description='Specify namespace of the robot')

    declare_y_1_position_cmd = DeclareLaunchArgument(
        'y_pose_1', default_value='0.5',
        description='Specify namespace of the robot')


    start_gazebo_ros_spawner_cmd_1 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', TURTLEBOT3_MODEL + '_1',
            '-file', urdf_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01'
        ],
        output='screen',
    )

    start_gazebo_ros_spawner_cmd_2 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', TURTLEBOT3_MODEL + '_2',
            '-file', urdf_path,
            '-y', y_pose_1,
            '-x', x_pose_1,
            '-z', '0.01'
        ],
        output='screen',
    )

    bridge_params = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'params',
        model_folder+'_bridge.yaml'
    )

    start_gazebo_ros_bridge_cmd_1 = GroupAction([
        PushRosNamespace('robot1'),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '--ros-args',
                '-p',
                f'config_file:={bridge_params}',
            ],
            output='screen',
        )
    ])

    start_gazebo_ros_bridge_cmd_2 = GroupAction([
        PushRosNamespace('robot2'),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '--ros-args',
                '-p',
                f'config_file:={bridge_params}',
            ],
            output='screen',
        )
    ])

    start_gazebo_ros_image_bridge_cmd_1 = GroupAction([
        PushRosNamespace('robot1'),
        Node(
            package='ros_gz_image',
            executable='image_bridge',
            arguments=['/camera/image_raw'],
            output='screen',
        )
    ])

    start_gazebo_ros_image_bridge_cmd_2 = GroupAction([
        PushRosNamespace('robot2'),
        Node(
            package='ros_gz_image',
            executable='image_bridge',
            arguments=['/camera/image_raw'],
            output='screen',
        )
    ])



    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)
    ld.add_action(declare_x_1_position_cmd)
    ld.add_action(declare_y_1_position_cmd)

    # Add any conditioned actions
    ld.add_action(start_gazebo_ros_spawner_cmd_1)
    ld.add_action(start_gazebo_ros_spawner_cmd_2)
    ld.add_action(start_gazebo_ros_bridge_cmd_1)
    ld.add_action(start_gazebo_ros_bridge_cmd_2)
    
    if TURTLEBOT3_MODEL != 'burger':
        ld.add_action(start_gazebo_ros_image_bridge_cmd_1)
        ld.add_action(start_gazebo_ros_image_bridge_cmd_2)


    return ld
