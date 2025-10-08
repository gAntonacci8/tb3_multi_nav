'''
Main launcher for multi robot tag-game (2 robots). 

Namespaces: robot1, robot2

Tentative approach to modular navigation2 nodes, manually configuring namespaces.
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
    #--- Coordinates: Robot1 (-3 -1 0.01 0 0 0), Robot2 (-3 1 0 0 0 0.01) 

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
            launch_arguments={'gz_args': f'-r {custom_world}','use_sim_time':'true'}.items() #maybe 'use_sim_time':'true' ? 
    )
    #Spawn robots in Gazebo
    # Spawn robot1
    default="/root/ros_ws/src/tb3_multi_nav/config/model.sdf"
    spawn_robot1 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'robot1', '-x', '-3', '-y', '-1', '-z', '0.01',
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
		#namespace='robot1',                                                #maybe not needed. Different name.
	arguments=['--ros-args', '-p', f'config_file:={robot1_bridge_yaml}'],
    	output='screen'
    )

    #robot urdf publisher for TF and Rviz
    robot1_state_publisher = Node(
	        	package='robot_state_publisher',
	        	executable='robot_state_publisher',
                name="robot1_state_publisher",
	        	namespace='robot1',                 #maybe not needed
	        	output='screen',
				    #remappings=[('tf', '/tf'), ('tf_static', '/tf_static')],
                parameters=[{
                    'use_sim_time': True,
                    'robot_description': Command([
                        'xacro ','/root/ros_ws/src/tb3_multi_nav/config/turtlebot3_waffle.urdf'
                        ' namespace:=robot1/' #space BEFORE "namespace" is needed. "/" after is needed.
                    ])
                }]
    )
    #------------------------ Nav2 stack nodes -------------------------------------------------------
    nav2_params_file1='/root/ros_ws/src/tb3_multi_nav/config/waffle.yaml'  #.yaml config file for nav2 nodes.    

    # Cartographer Node (SLAM: map -> odom) and occupancy_grid node 
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
        # Argomenti aggiuntivi per il file di configurazione .lua
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

    # AMCL (Adaptive Monte Carlo Localization) node. Same function (TF map -> odom frames) as cartographer. Mutually exclusive with it
    amcl_node1=LifecycleNode(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        namespace='robot1', #robot1 for multi-robot scenario
        output='screen',
        parameters=[
            {'use_sim_time':True},
            nav2_params_file1]
            ,
        remappings=[  #("topic to redirect" --> "topic destination")
                    ("/robot1/map", "/map"), 
                    ("/robot1/map_updates","/map_updates"),
                    ("/scan","/robot1/scan"),
                    ('/initialpose', '/robot1/initialpose'),
                    ('/amcl_pose', '/robot1/amcl_pose'),
                ]
    )

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
    # Controller Server (Lifecycle)
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
    # '''Addressing error in bt_navigator 'can't find XML file: overwrite in XML with the specific key '''
    # bt_package_share = FindPackageShare('nav2_bt_navigator')

    # behavior_tree_xml = PathJoinSubstitution([
    #     bt_package_share, 
    #     'behavior_trees', 
    #     'navigate_to_pose_w_replanning_and_recovery.xml'
    # ])
    bt_navigator_node = LifecycleNode(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        namespace="robot1",   
        output='screen',
        parameters=[
            { 'use_sim_time': True},
            nav2_params_file1,
           # {'global_frame': 'map'},
           # {'robot_base_frame': ['robot1', '/base_link']} # Forzato con la sintassi di concatenazione]
        ], 
        remappings=[
            ("/robot1/map", "/map"), ("/robot1/map_updates","/map_updates")
        ]
    )
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
    #explorer node (from AniArka repo) 
    #NB repo does ros2 run custom_explorer explorer, no arguments. Set output screen to get it
    # explorer_node=Node(
    #     package="custom_explorer",
    #     executable="explorer",
    #     output="screen"
    # )
    #map selector, overrid the "yaml_filename" parameter in "waffle.yaml". Just for simplicity.
    map_yaml_selector=[ 
        '/root/ros_ws/src/tb3_multi_nav/utils/newmaze.yaml',
        '/root/ros_ws/src/tb3_multi_nav/utils/2ndattempt_newmaze.yaml'  #actually using this one. Better mapping.

    ]
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
    smoother_server1=LifecycleNode(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        namespace="robot1", #robot1 for multi
        parameters=[nav2_params_file1, {'use_sim_time': True}],
        remappings=[
            ("/robot1/map", "/map"), ("/robot1/map_updates","/map_updates")
        ]
    )
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
    # Activation order is CRUCIAL.
    lifecycle_manager = TimerAction(
        period=2.0,
        actions=[Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager1',
        namespace="robot1",         #maybe needed
        output='screen',
        parameters=[  
            {'use_sim_time':True},
            {'autostart': True},    #starts all nodes automatically
            {'node_names': [        #nodes list
                #'map_server',      #commented to avoid cartographer conflicts. First when using already mapped envs.
                'amcl',             #testing if lifecylce, as other Nav2 nodes
                'behavior_server',
                'smoother_server',
                'planner_server','controller_server',
                'velocity_smoother','bt_navigator',
                'waypoint_follower' #-------------- WAS COMMENTED. Works fine uncommented.
            ]}
        ])] 
    )

    #---------------- Rviz2 and configuration ----------------------------
    # Sets pose for robot1 in Rviz automatically. Single publish, no node
    init_pose_robot1 =TimerAction( 
        period=6.0,
        actions=[
        ExecuteProcess(
			cmd=[
				'ros2', 'topic', 'pub', '--once',
				#'/robot1/initialpose', #uncomment with 2 robots. "/initialpose" for single robot.
                '/robot1/initialpose',
				'geometry_msgs/PoseWithCovarianceStamped',
				'{header: {frame_id: "map"}, pose: {pose: {position: {x: -3, y: -1, z: 0.01}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}}'
			],
			output='screen'
        )]
    )
	
    default_rviz_config_path = os.path.join(
        get_package_share_directory('nav2_bringup'), 
        'rviz', 
        'nav2_default_view.rviz'
    )
    ''' bugged-- 3 /rviz nodes instances created
    #rviz node -- last to launch
    rviz_node=Node(
        package='rviz2',
        executable='rviz2',
        #name='rviz',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}]
    )  
    '''
    rviz_config_path="/root/ros_ws/src/tb3_multi_nav/config/rviz_namespaced_singleBot.rviz" 
    # rviz_node = ExecuteProcess(
    #     cmd=['rviz2', '-d', rviz_config_path, '--ros-args', '-p', 'use_sim_time:=True'],
    #     #name='rviz_visualizer', 
    #     output='screen',
    #     shell=True                      # Executes like shell
    # )
    rviz_node=Node(
        package='rviz2',
        executable='rviz2',
        #name='rviz',         # BAD BEHAVIOR, multi nodes with same name. Issue: https://github.com/ros2/rviz/issues/671 
        namespace="robot1",  #fixes nav2 selector issue, still BAD BEHAVIOR.
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}]
        ,
        remappings=[            
              
            
        ]
    ) 
    
    #Launches nodes in the exact order. for Lifecycles nodes, order not important here, but in the Manager
    return LaunchDescription([          
        gazebo,
        spawn_robot1,
        #spawn_robot2,          #not yet
        bridge_robot1,
        robot1_state_publisher,

        #cartographer_node1,     # comment when migrating to AMCL
        #occupancy_grid1,        # comment when migrating to AMCL (maybe?)
        
        amcl_node1,             # comment when using cartographer

        planner_server_node1,
        controller_server_node1,
        behavior_server_node1,
        smoother_server1,
        velocity_smoother_node1,
        bt_navigator_node,
        waypoint_follower_node1,
        map_server_node,        # conflict when using cartographer. Uncomment when migrating to AMCL and .yaml done.
        map_server_manager,     #separate manager to launch global node (only map)
        #explorer_node,         # autonomous mapping external node. Not properly working or missing nav2 stuff.
        lifecycle_manager,
        rviz_node,
        #init_pose_robot1        # Also set on waffle.yaml, AMCL section.  
    ])
