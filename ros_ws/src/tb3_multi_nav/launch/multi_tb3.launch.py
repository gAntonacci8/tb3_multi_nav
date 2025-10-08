'''
Main launcher for multi robot tag-game (2 robots). 

Namespaces: "robot1", "robot2".

Tentative approach to modular navigation2 nodes, manually configuring namespaces. 

Coding logic

Zone1: Gazebo logic and robots spawn in it

Zone2: Nav2 stack bringup, one per robot

Zone3: Double Rviz logic

Zone4: Nodes launching

'''
from launch import LaunchDescription
from launch.actions import ExecuteProcess,TimerAction, IncludeLaunchDescription, GroupAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LifecycleNode, Node, PushRosNamespace
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    namespaces=['robot1','robot2']

    #------------ Open Gazebo and spawn two (2) turtlebot3 waffle
    #--- Coordinates: Robot1 (-3 -1 0.01 0 0 0), Robot2 (-3 1 0.01 0 0 0) 

    #get custom maze worldfor gazebo
    custom_world= "/root/ros_ws/src/tb3_multi_nav/utils/maze_world.world" 

    #Launch Gazebo 
    gazebo = IncludeLaunchDescription(                     
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': f'-r {custom_world}','use_sim_time':'true'}.items()  
    )
    #Spawn robots in Gazebo
    #SDF files
    robot1_sdf="/root/ros_ws/src/tb3_multi_nav/config/robot1_model.sdf"
    robot2_sdf="/root/ros_ws/src/tb3_multi_nav/config/robot2_model.sdf"
    # Spawn robot1
    spawn_robot1 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'robot1', '-x', '-3', '-y', '-1', '-z', '0.01',
                   '-file', robot1_sdf],
        output='screen'
    )

    # Spawn robot2
    spawn_robot2 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'robot2', '-x', '-3', '-y', '1', '-z', '0.01',
                   '-file', robot2_sdf], 
        output='screen'
    )
    #--------------- Bridges for ROS-GZ communications ------------------------
    # --- YAML Bridge files ---
    robot1_bridge_yaml='/root/ros_ws/src/tb3_multi_nav/config/robot1_turtlebot3_waffle_bridge.yaml'
    robot2_bridge_yaml='/root/ros_ws/src/tb3_multi_nav/config/robot2_turtlebot3_waffle_bridge.yaml'
    # --- Bridge robot1 ---
    bridge_robot1 = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge1',                  # namespace not needed. Different name.
        parameters=[{'use_sim_time': True}],
        arguments=['--ros-args', '-p', f'config_file:={robot1_bridge_yaml}'],
        output='screen'
    )

    bridge_robot2 = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge2',                  # namespace not needed. Different name.
        parameters=[{'use_sim_time': True}],
        arguments=['--ros-args', '-p', f'config_file:={robot2_bridge_yaml}'],
            output='screen'
    )
    #--------------- Robot State Publishers for TF and topic communications ------------------------
    #NB: Single file due to XACRO presence. 
    urdf_file_path='/root/ros_ws/src/tb3_multi_nav/config/turtlebot3_waffle.urdf'
    #robot urdf publisher for TF and Rviz
    robot1_state_publisher = Node(
	        	package='robot_state_publisher',
	        	executable='robot_state_publisher',
                name="robot1_state_publisher",
	        	namespace='robot1',                 #maybe not needed, but not a problem.
	        	output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'robot_description': Command([
                        'xacro ',urdf_file_path,
                        ' namespace:=robot1/'       #space BEFORE "namespace" is needed. "/" after is needed.
                    ])
                }]
    )
    robot2_state_publisher = Node(
	        	package='robot_state_publisher',
	        	executable='robot_state_publisher',
                name="robot2_state_publisher",
	        	namespace='robot2',                  #maybe not needed, but not a problem.
	        	output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'robot_description': Command([
                        'xacro ',urdf_file_path,
                        ' namespace:=robot2/'        #space BEFORE "namespace" is needed. "/" after is needed.
                    ])
                }]
    )
    #------------------------ Nav2 stack nodes -------------------------------------------------------
    # For multi robot scenario, every node will be create in pairs: one for "robot1", the other for "robot2".
    # Instances will be coded one after the other, keeping the nav2 logic at the same code zones.
    # NB lifecycle manager will be doubled as well, one per namespace, as lifecycle node MUST keep their names
    # as default.

    #.yaml config file for nav2 nodes.  
    nav2_params_file1='/root/ros_ws/src/tb3_multi_nav/config/robot1_waffle.yaml'    #for robot1
    nav2_params_file2='/root/ros_ws/src/tb3_multi_nav/config/robot2_waffle.yaml'    #for robot2

    # Cartographer Node (SLAM: map -> odom) + occupancy_grid node 
    # !!!!!!!!!! WARNING: Nodes NOT namespaced. Not tested in namespaced scenario. Left here for convenience.
    # DO NOT USE THESE NODES "AS IS". Check namespaces and their correlations before use.
    # IMP: These nodes are mutually exclusive with AMCL node. Cartographer also overwrites map_server in /map topic, so 
    # preloaded map is overrun and useless. 
    # Doc: https://ros2-industrial-workshop.readthedocs.io/en/latest/_source/navigation/ROS2-Cartographer.html 
    # package dir: '/opt/ros/jazzy/share/cartographer_ros/, config folder: configuration_files/
    cartographer_node1 = Node(
        package='cartographer_ros', # O turtlebot3_cartographer se usi il loro wrapper
        executable='cartographer_node',
        name='cartographer_node1',
        output='screen',
        parameters=[{'use_sim_time': True}],
        # Additional ags for ".lua" config file.
        arguments=[
            '-configuration_directory', '/root/ros_ws/src/turtlebot3-jazzy/turtlebot3_cartographer/config',
            '-configuration_basename', 'turtlebot3_lds_2d.lua'
        ]
    )
    occupancy_grid1=Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'] #values from documentation
    )

    # AMCL (Adaptive Monte Carlo Localization) nodes (Lifecycle). 
    # Same function (TF map -> odom frames) as cartographer. Mutually exclusive with it.

    #AMCL robot1
    amcl_node1=LifecycleNode(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        namespace='robot1', #robot1 for multi-robot scenario
        output='screen',
        parameters=[
            {'use_sim_time':True},
            nav2_params_file1
        ],
        remappings=[  #("topic to redirect" --> "topic destination")
            ("/robot1/map", "/map"), 
            ("/robot1/map_updates","/map_updates"),
            ("/scan","/robot1/scan"),
            ('/initialpose', '/robot1/initialpose'),
            ('/amcl_pose', '/robot1/amcl_pose'),
        ]
    )
    #AMCL robot2
    amcl_node2=LifecycleNode(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        namespace='robot2', #robot1 for multi-robot scenario
        output='screen',
        parameters=[
            {'use_sim_time':True},
            nav2_params_file2
        ],
        remappings=[  #("topic to redirect" --> "topic destination")
            ("/robot2/map", "/map"), 
            ("/robot2/map_updates","/map_updates"),
            ("/scan","/robot2/scan"),
            ('/initialpose', '/robot2/initialpose'),
            ('/amcl_pose', '/robot2/amcl_pose'),
        ]
    )

    #Planner Servers (Lifecycle)

    #Planner server robot1
    planner_server_node1 = LifecycleNode(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        namespace="robot1",
        parameters=[nav2_params_file1, {'use_sim_time': True}],
        output='screen',
        remappings=[
            ("/robot1/map", "/map"), ("/robot1/map_updates","/map_updates")
        ]
    )
    #Planner Server robot2
    planner_server_node2 = LifecycleNode(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        namespace="robot2",
        parameters=[nav2_params_file2, {'use_sim_time': True}],
        output='screen',
        remappings=[
            ("/robot2/map", "/map"), ("/robot2/map_updates","/map_updates")
        ]
    )
    # Controller Servers (Lifecycle)
    #Controller Server robot1
    controller_server_node1 = LifecycleNode(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        namespace="robot1",        
        parameters=[nav2_params_file1, {'use_sim_time': True}],
        output='screen',
        remappings=[
            ("/robot1/map", "/map"), ("/robot1/map_updates","/map_updates")
        ]
    )
    #Controller Server robot2
    controller_server_node2 = LifecycleNode(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        namespace="robot2",        
        parameters=[nav2_params_file2, {'use_sim_time': True}],
        output='screen',
        remappings=[
            ("/robot2/map", "/map"), ("/robot2/map_updates","/map_updates")
        ]
    )

    bt_navigator_node1 = LifecycleNode(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        namespace="robot1",   
        output='screen',
        parameters=[
            { 'use_sim_time': True},
            nav2_params_file1,
        ], 
        remappings=[
            ("/robot1/map", "/map"), ("/robot1/map_updates","/map_updates")
        ]
    )
    bt_navigator_node2 = LifecycleNode(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        namespace="robot2",   
        output='screen',
        parameters=[
            { 'use_sim_time': True},
            nav2_params_file2,
        ], 
        remappings=[
            ("/robot2/map", "/map"), ("/robot2/map_updates","/map_updates")
        ]
    )

    #Behavior Servers (Lifecycle)
    behavior_server_node1 = LifecycleNode(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        namespace="robot1",
        output='screen',
        parameters=[{'use_sim_time': True}, nav2_params_file1],
        remappings=[
            ("/robot1/map", "/map"), 
            ("/robot1/map_updates","/map_updates"),
            ('/local_costmap/costmap_raw','/robot1/local_costmap/costmap_raw') ,         
            ('/local_costmap/costmap_raw_updates', '/robot1/local_costmap/costmap_raw_updates'), 
            ('/local_costmap/published_footprint', '/robot1/local_costmap/published_footprint')
        ]
    )

    behavior_server_node2 = LifecycleNode(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        namespace="robot2",
        output='screen',
        parameters=[{'use_sim_time': True}, nav2_params_file2],
        remappings=[
            ("/robot2/map", "/map"), 
            ("/robot2/map_updates","/map_updates"),
            ('/local_costmap/costmap_raw','/robot2/local_costmap/costmap_raw') ,         
            ('/local_costmap/costmap_raw_updates', '/robot2/local_costmap/costmap_raw_updates'), 
            ('/local_costmap/published_footprint', '/robot2/local_costmap/published_footprint')
        ]
    )
    #map selector, overrid the "yaml_filename" parameter in "waffle.yaml". Just for simplicity.
    map_yaml_selector=[ 
        '/root/ros_ws/src/tb3_multi_nav/utils/newmaze.yaml',
        '/root/ros_ws/src/tb3_multi_nav/utils/2ndattempt_newmaze.yaml'  #actually using this one. Better mapping.
    ]
    # IMP: Map server is GLOBAL. Even in multi robot, single map provider.
    map_server_node = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        namespace="",            #no namespace, global map.
        name='map_server',
        parameters=[nav2_params_file1, {'use_sim_time': True, 'yaml_filename':map_yaml_selector[1]}],
        output='screen'
    )
    map_server_manager=Node(     
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='map_lifecycle_manager',
        namespace="",
        parameters=[
            {'use_sim_time':True},
            {'autostart': True},    #starts all nodes automatically
            {'node_names': [        #nodes list
                'map_server'      
            ]}
        ]
    )
    #Smoother Servers (Lifecycle)
    smoother_server1=LifecycleNode(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        namespace="robot1", 
        parameters=[nav2_params_file1, {'use_sim_time': True}],
        remappings=[
            ("/robot1/map", "/map"), ("/robot1/map_updates","/map_updates")
        ]
    )
    smoother_server2=LifecycleNode(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        namespace="robot2", #robot1 for multi
        parameters=[nav2_params_file2, {'use_sim_time': True}],
        remappings=[
            ("/robot2/map", "/map"), ("/robot2/map_updates","/map_updates")
        ]
    )
    #Velocity Smoothers (Lifecycle)
    # ------> Still evaluating if cmd_vel should be remapped.
    velocity_smoother_node1 = LifecycleNode(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',  
        output='screen',
        namespace="robot1",
        parameters=[
            nav2_params_file1,
            {'use_sim_time': True}
        ],
        remappings=[
            ("/robot1/map", "/map"), ("/robot1/map_updates","/map_updates")
            #,('cmd_vel', 'cmd_vel_smoothed')
        ]
        # Questo nodo riceve i comandi dal controller e li reindirizza al robot.
        # Assicurati che il tuo controller pubblichi su /cmd_vel_nav e il tuo robot ascolti su /cmd_vel.
        # Se il tuo controller e il tuo robot usano /cmd_vel di default, potresti aver bisogno di remapping:
        # ,remappings=[
        #     ('cmd_vel', 'cmd_vel_smoothed')
        # ]
    )
        #Velocity Smoothers (Lifecycle)
    velocity_smoother_node2 = LifecycleNode(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',  
        output='screen',
        namespace="robot2",
        parameters=[
            nav2_params_file2,
            {'use_sim_time': True}
        ],
        remappings=[
            ("/robot2/map", "/map"), ("/robot2/map_updates","/map_updates")
            #,('cmd_vel', 'cmd_vel_smoothed')
        ]
        # Questo nodo riceve i comandi dal controller e li reindirizza al robot.
        # Assicurati che il tuo controller pubblichi su /cmd_vel_nav e il tuo robot ascolti su /cmd_vel.
        # Se il tuo controller e il tuo robot usano /cmd_vel di default, potresti aver bisogno di remapping:
        # ,remappings=[
        #     ('cmd_vel', 'cmd_vel_smoothed')
        # ]
    )

    #Waypoint Follower (Lifecycle)
    waypoint_follower_node1 = LifecycleNode(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',  
        output='screen',
        namespace="robot1",       
        parameters=[
            nav2_params_file1,
            {'use_sim_time': True}
        ],
        remappings=[
            ("/robot1/map", "/map"), ("/robot1/map_updates","/map_updates")
        ]
    )
    waypoint_follower_node2 = LifecycleNode(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',  
        output='screen',
        namespace="robot2",       
        parameters=[
            nav2_params_file2,
            {'use_sim_time': True}
        ],
        remappings=[
            ("/robot2/map", "/map"), ("/robot2/map_updates","/map_updates")
        ]
    )
    lifecycle_nodes = [             #Nav2. For check. NOT IN ORDER.
        'controller_server', #ok
        'smoother_server',  #ok
        'planner_server', #ok
        'behavior_server', #ok
        'bt_navigator', #ok
        'waypoint_follower', #ok
        'velocity_smoother' #ok
    ]
    # Lifecycle Manager-- activates all lifecycle nodes -- needed for all nav2 nodes
    # Cloned istances. Same data, execpt for namespace.
    # Activation order is CRUCIAL.

    #Manager for robot1 Nav2 stack nodes 
    lifecycle_manager1 = TimerAction(
        period=2.0,
        actions=[Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager1',
        namespace="robot1",         
        output='screen',
        parameters=[  
            {'use_sim_time':True},
            {'autostart': True},    #starts all nodes automatically
            {'node_names': [        #nodes list
                'amcl',             
                'behavior_server',
                'smoother_server',
                'planner_server',
                'controller_server',
                'velocity_smoother',
                'bt_navigator',
                'waypoint_follower' 
            ]}
        ])] 
    )
    #Manager for robot2 Nav2 stack nodes 
    lifecycle_manager2 = TimerAction(
        period=2.0,
        actions=[Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager2',
        namespace="robot2",         
        output='screen',
        parameters=[  
            {'use_sim_time':True},
            {'autostart': True},    #starts all nodes automatically
            {'node_names': [        #nodes list
                'amcl',             
                'behavior_server',
                'smoother_server',
                'planner_server',
                'controller_server',
                'velocity_smoother',
                'bt_navigator',
                'waypoint_follower' 
            ]}
        ])] 
    )

    #---------------- Rviz2 and configuration ----------------------------
    # Since Nav2 stack is namespaced, going with 2 Rviz2 windows route.
    # Each windows should be able to monitor both robots, but (IDEALLY) robot1 can be 
    # controlled only by Rviz2 window1, the other only by Rviz2 window2, as Nav2 stacks etc are bounder
    # by namespaces. 

    # Two configs file, one per Rviz view.
    default_rviz_config_path = os.path.join(                #default config path for backup
        get_package_share_directory('nav2_bringup'), 
        'rviz', 
        'nav2_default_view.rviz'
    )
    rviz_config_path1="/root/ros_ws/src/tb3_multi_nav/config/robot1_rviz_namespaced.rviz" 
    rviz_config_path2="/root/ros_ws/src/tb3_multi_nav/config/robot2_rviz_namespaced.rviz" 

    #Rviz2 window for robot1 control.
    rviz_node1=Node(
        package='rviz2',
        executable='rviz2',
        #name='rviz',         # BAD BEHAVIOR, multi nodes with same name. Issue: https://github.com/ros2/rviz/issues/671 
        namespace="robot1",  #fixes nav2 selector issue. Nav2 online.
        output='screen',
        arguments=['-d', rviz_config_path1],
        parameters=[{'use_sim_time': True}],
        remappings=[   #no remapping needed so far. Kept as it doesn't cause issues.         
                  
        ]
    )
    #Rviz2 windows for robot2 control.
    rviz_node2=Node(
        package='rviz2',
        executable='rviz2',
        #name='rviz',         # BAD BEHAVIOR, multi nodes with same name. Issue: https://github.com/ros2/rviz/issues/671 
        namespace="robot2",  #fixes nav2 selector issue. Nav2 online.
        output='screen',
        arguments=['-d', rviz_config_path2],
        parameters=[{'use_sim_time': True}],
        remappings=[   #no remapping needed so far. Kept as it doesn't cause issues.         
                  
        ]
    ) 
    
    #Launches nodes in the exact order. for Lifecycles nodes, order not important here, but in the Manager
    return LaunchDescription([          
        gazebo,
        spawn_robot1,
        spawn_robot2,

        bridge_robot1,
        bridge_robot2,
        
        robot1_state_publisher,
        robot2_state_publisher,

        #cartographer_node1,     # comment when migrating to AMCL
        #occupancy_grid1,        # comment when migrating to AMCL (maybe?)
        
        amcl_node1,             # comment when using cartographer
        amcl_node2,

        planner_server_node1,
        planner_server_node2,

        controller_server_node1,
        controller_server_node2,

        behavior_server_node1,
        behavior_server_node2,

        smoother_server1,
        smoother_server2,

        velocity_smoother_node1,
        velocity_smoother_node2,

        bt_navigator_node1,
        bt_navigator_node2,

        waypoint_follower_node1,
        waypoint_follower_node2,

        map_server_node,        # conflict when using cartographer. Uncomment when migrating to AMCL and .yaml done.
        map_server_manager,     #separate manager to launch global node (only map)
        
        lifecycle_manager1,
        lifecycle_manager2,

        rviz_node1,
        rviz_node2
    ])
