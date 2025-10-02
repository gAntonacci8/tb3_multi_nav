'''
Main launcher for multi robot tag-game (2 robots). 

Namespaces: robot1, robot2

Tentative approach to modular navigation2 nodes, manually configuring namespaces.
'''
from launch import LaunchDescription
from launch.actions import ExecuteProcess,TimerAction, IncludeLaunchDescription, GroupAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LifecycleNode,Node, PushRosNamespace
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    namespaces=['robot1','robot2']

    #------------ Open Gazebo and spawn two (2) turtlebot3 waffle
    #--- Coordinates: Robot1 (-0.5 0 0.01 0 0 0), Robot2 (-0.5 -1 0 0 0 0.01) 

    #get STANDARD turtlebot3 world for gazebo
    tb3_world = os.path.join(                               
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds', 'turtlebot3_world.world'
    )
    #get custom maze worldfor gazebo
    custom_world= "/root/ros_ws/src/tb3_multi_nav/utils/maze_world.world" 
    model_folder="turtlebot3_waffle"    
    #sdf path to waffle robot file                    
    sdf_path = os.path.join(                                
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
            launch_arguments={'gz_args': f'-r {custom_world}'}.items() #maybe 'use_sim_time':'true' ? 
    )
    #Spawn robots in Gazebo
    # Spawn robot1
    default="/root/ros_ws/src/tb3_multi_nav/config/model.sdf"
    spawn_robot1 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', '', '-x', '-0.5', '-y', '0', '-z', '0.01',
                   '-file', default],#'/root/ros_ws/src/tb3_multi_nav/config/model_robot1.sdf'], #sdf_path BACKUP
        output='screen'
    )

    # Spawn robot2
    spawn_robot2 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'robot2', '-x', '-0.5', '-y', '1', '-z', '0.01',
                   '-file', '/root/ros_ws/src/tb3_multi_nav/config/model_robot2.sdf'], #sdf_path BACKUP
        output='screen'
    )
    #--------------- Bridges for ROS-GZ communications ------------------------
    # --- Bridge robot1 ---
    robot1_bridge_yaml='/root/ros_ws/src/tb3_multi_nav/config/turtlebot3_waffle_bridge.yaml'
    bridge_robot1 = Node(
	package='ros_gz_bridge',
	executable='parameter_bridge',
    	name='ros_gz_bridge1',
        parameters=[{'use_sim_time': True}],

		#namespace='robot1',
	arguments=['--ros-args', '-p', f'config_file:={robot1_bridge_yaml}'],
    	output='screen'
    )

    #robot urdf publisher for TF and Rviz
    robot1_state_publisher = Node(
	        	package='robot_state_publisher',
	        	executable='robot_state_publisher',
	        	#namespace='robot1',
	        	output='screen',
				    #remappings=[('tf', '/tf'), ('tf_static', '/tf_static')],
                parameters=[{
                    'use_sim_time': True,
                    'robot_description': Command([
                        'xacro ','/root/ros_ws/src/tb3_multi_nav/config/turtlebot3_waffle.urdf'
                        #' namespace:=robot1/' #space BEFORE "namespace" is needed. "/" after is needed.
                    ])
                }]
    )
    #------------------------ Nav2 stack nodes -------------------------------------------------------
    static_tf_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_footprint_base_link_tf',
    output='screen',
    # [x, y, z, roll, pitch, yaw, parent_frame, child_frame]
    # Spostamento tipico del base_link di 1 cm sopra base_footprint
    arguments=['0', '0', '0.01', '0', '0', '0', 'base_footprint', 'base_link']
    )
    #Cartographer Node (SLAM: map -> odom)
    # package dir: '/opt/ros/jazzy/share/cartographer_ros/, config folder: configuration_files/
    cartographer_node1 = Node(
        package='cartographer_ros', # O turtlebot3_cartographer se usi il loro wrapper
        executable='cartographer_node',
        name='cartographer_node1',
        output='screen',
        parameters=[{'use_sim_time': True}],
        # Argomenti aggiuntivi per il file di configurazione .lua
        arguments=[
            '-configuration_directory', '/root/ros_ws/src/turtlebot3-jazzy/turtlebot3_cartographer/config',
            '-configuration_basename', 'turtlebot3_lds_2d.lua'
        ]
    )
    nav2_params_file1='/root/ros_ws/src/tb3_multi_nav/config/waffle.yaml'
    planner_server_node1 = LifecycleNode(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        namespace="",
        parameters=[nav2_params_file1, {'use_sim_time': True}],
        output='screen'
    )
    # Controller Server (Lifecycle)
    controller_server_node1 = LifecycleNode(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        namespace="",        
        parameters=[nav2_params_file1, {'use_sim_time': True}],
        output='screen'
    )
    bt_navigator_node = LifecycleNode(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        namespace="",   
        output='screen',
        # Assicurati che use_sim_time sia True e che punti al tuo file di configurazione
        parameters=[{'use_sim_time': True}, nav2_params_file1]
    )
    behavior_server_node1 = LifecycleNode(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        namespace="",
        output='screen',
        parameters=[{'use_sim_time': True}, nav2_params_file1]
    )
    #explorer node (from AniArka repo) 
    #NB repo does ros2 run custom_explorer explorer, no arguments. Set output screen to get it
    explorer_node=Node(
        package="custom_explorer",
        executable="explorer",
        output="screen"
    )
    map_server_node = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        namespace="",
        name='map_server',
        parameters=[nav2_params_file1, {'use_sim_time': True}],
        output='screen'
    )
    # Lifecycle Manager-- activates all lifecycle nodes -- needed for all nav2 nodes
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[
            {'autostart': True},    #starts all nodes automatically
            {'node_names': [        #nodes list
                'planner_server','controller_server','bt_navigator','behavior_server'
            ]}
        ]
    ) 

    #---------------- Rviz2 and configuration ----------------------------
    rviz_config_path = os.path.join(
        get_package_share_directory('nav2_bringup'), 
        'rviz', 
        'nav2_default_view.rviz'
    )
    ''' bugged-- 3 /rviz nodes instances created
    #rviz node -- last to launch
    rviz_node=Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}]
    )  
    ''' 
    rviz_node = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config_path, '--ros-args', '-p', 'use_sim_time:=True'],
        name='rviz_visualizer',  # Aggiungi un nome unico per chiarezza
        output='screen',
        shell=True # Esegue il comando come se fosse in un terminale
    )
    
    return LaunchDescription([          #Launches nodes in the exact order
        gazebo,
        spawn_robot1,
        #spawn_robot2,
        bridge_robot1,
        robot1_state_publisher,
        cartographer_node1,
        #planner_server_node1,
        #controller_server_node1,
        #behavior_server_node1,
        #bt_navigator_node,
        #explorer_node,
        #lifecycle_manager,
        static_tf_node,
        rviz_node
    ])
