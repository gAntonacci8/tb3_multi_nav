from launch import LaunchDescription
from launch.actions import ExecuteProcess,TimerAction, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LifecycleNode,Node, PushRosNamespace
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    tb3_world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds', 'turtlebot3_world.world'
    )

    #path sdf and URDF --  BACKUP IT WORKS
    model_folder="turtlebot3_waffle"
    sdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        model_folder,
        'model.sdf'
    )
    urdf_file = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf',
        'turtlebot3_waffle.urdf'
    )
    #extract robot description for publishers
    with open(urdf_file, 'r') as infp:
        urdf_path = infp.read()

    #Waffle yaml file configured for nav2
    nav2_waffle_yaml1='/root/ros_ws/src/tb3_multi_nav/config/nav2_waffle_robot1.yaml'
    nav2_waffle_yaml2='/root/ros_ws/src/tb3_multi_nav/config/nav2_waffle_robot2.yaml'


    #Fake clock to publish sim time 
    fake_clock_node = Node(
    	package='tb3_multi_nav',  
    	executable='fake_clock',
    	name='fake_clock',
    	output='screen'
    )
    # Open gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {tb3_world}'}.items()
    )
    # Spawn robot1
    spawn_robot1 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'robot1', '-x', '-0.5', '-y', '0', '-z', '0.01',
                   '-file', '/root/ros_ws/src/tb3_multi_nav/config/model_robot1.sdf'], #sdf_path BACKUP
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
    # TEST to get Rviz2 work -- NEEDS URDF NOT SDF
    robot1_state_publisher = TimerAction(
	period=2.0, #wait 2 sec for clock	
	actions=[Node(
	        	package='robot_state_publisher',
	        	executable='robot_state_publisher',
	        	namespace='robot1',
	        	output='screen',
			parameters=[{
			                'use_sim_time': True,
			                'robot_description': Command([
			                    'xacro ',
			                    os.path.join(get_package_share_directory('turtlebot3_description'), 
			                                 'urdf', 'turtlebot3_waffle.urdf'),
			                    ' namespace:=robot1/' #space BEFORE "namespace" is needed. "/" after is needed.
			                ])
			            }]
		)]
    )
    robot2_state_publisher =TimerAction(
	period=2.0, #wait 2 sec for clock 
	actions=[Node(
		        package='robot_state_publisher',
		        executable='robot_state_publisher',
		        namespace='robot2',
		        output='screen',
			parameters=[{
			                'use_sim_time': True,
			                'robot_description': Command([
			                    'xacro ',
			                    os.path.join(get_package_share_directory('turtlebot3_description'), 
			                                 'urdf', 'turtlebot3_waffle.urdf'),
			                    ' namespace:=robot2/' #space BEFORE "namespace" is needed. "/" after is needed.
			                ])
			            }]
		 )]
    )
	# Static TF map -> odom for robot1
    map_to_odom_robot1 =TimerAction(
	period=2.0, #wait 2 sec for clock 
	actions=[ GroupAction([
		        Node(
		            package='tf2_ros',
		            executable='static_transform_publisher',
		            #name='map_to_odom1',
					name='transformer1',
		            output='screen',
		            arguments=['-0', '0', '0', '0', '0', '0', 'robot1/base_footprint', 'robot1/base_link']
		        )])
    	]
    )
    # Static TF map -> odom per robot2
    map_to_odom_robot2 =TimerAction(
	period=2.0, #wait 2 sec for clock 
	actions=[ GroupAction([
		        Node(
		            package='tf2_ros',
		            executable='static_transform_publisher',
		            #name='map_to_odom2',
		            name='transformer2',
					output='screen',
		            arguments=['-0', '0', '0', '0', '0', '0', 'robot2/base_footprint', 'robot2/base_link']
		        )
		    ])
		]
    )



    #Link TF from Odom to Base_footprint of each robot
    odom_to_base_robot1=TimerAction(
	period=2.0, #wait 2 sec for clock 
	actions=[ GroupAction([
		        Node(
		            package='tf2_ros',
		            executable='static_transform_publisher',
		            name='odom_to_base_footprint_robot1',
		            output='screen',
		            arguments=['0', '0', '0', '0', '0', '0', 'robot1/odom', 'robot1/base_footprint']
		        )
		    ])
		]
    )
    odom_to_base_robot2=TimerAction(
	period=2.0, #wait 2 sec for clock 
	actions=[ GroupAction([
		        Node(
		            package='tf2_ros',
		            executable='static_transform_publisher',
		            name='odom_to_base_footprint_robot2',
		            output='screen',
		            arguments=['0', '0', '0', '0', '0', '0', 'robot2/odom', 'robot2/base_footprint']
		        )
		    ])
		]
    )

    broadcaster_odom_robot1=TimerAction(
		period=2.0, #wait 2 sec for clock 
		actions=[Node(
					package='tb3_multi_nav',
					executable='broadcaster_odom',
					name='odom_tf_broadcaster1',
					namespace='robot1',
					output='screen'
		)]
	)
    broadcaster_odom_robot2=TimerAction(
		period=2.0, #wait 2 sec for clock 
		actions=[Node(
					package='tb3_multi_nav',
					executable='broadcaster_odom',
					name='odom_tf_broadcaster2',
					namespace='robot2',
					output='screen'
		)]
	)

    # load config files YAML for both robots
    robot1_bridge_yaml = os.path.join(
        get_package_share_directory('tb3_multi_nav'),
        'config', 'robot1_bridge.yaml'
    )

    robot2_bridge_yaml = os.path.join(
        get_package_share_directory('tb3_multi_nav'),
        'config', 'robot2_bridge.yaml'
    )
    # --- Bridge robot1 ---
    bridge_robot1 = Node(
	package='ros_gz_bridge',
	executable='parameter_bridge',
    	name='ros_gz_bridge1',
		namespace='robot1',
	arguments=['--ros-args', '-p', f'config_file:={robot1_bridge_yaml}'],
    	output='screen'
    )
    # --- Bridge robot2 ---
    bridge_robot2 = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
		namespace='robot2',
	name='ros_gz_bridge2',
    	arguments=['--ros-args', '-p', f'config_file:={robot2_bridge_yaml}'],
        output='screen'
    )


    map_yaml_file = '/root/ros_ws/install/turtlebot3_navigation2/share/turtlebot3_navigation2/map/map.yaml'
    map_server = LifecycleNode(
	    package='nav2_map_server',
	    executable='map_server',
	    name='map_server',
	    namespace='',
	    output='screen',
	    parameters=[{
			'yaml_filename': map_yaml_file, 
			'use_sim_time': True,
			'map_qos_profile': {
				'durability': 'transient_local',
				'depth': 1
        	}
		}]
    )
    # Map server for Rviz2 and Nav2 -- GENERAL, LIFECYCLE NODE.  Needs activiation
    map_server1 = LifecycleNode(
	    package='nav2_map_server',
	    executable='map_server',
	    name='map_server',
	    namespace='robot1',
	    output='screen',
	    parameters=[{
			'yaml_filename': map_yaml_file, 
			'use_sim_time': True,
			'map_qos_profile': {
				'durability': 'transient_local',
				'depth': 1
        	}
		}]
    )
    map_server2 = LifecycleNode(
		package='nav2_map_server',
		executable='map_server',
		name='map_server',
		namespace='robot2',
		output='screen',
		parameters=[{
			'yaml_filename': map_yaml_file, 
			'use_sim_time': True,
			'map_qos_profile': {
				'durability': 'transient_local',
				'depth': 1
			}
		}]
    )
	#configures and activates lifecycle node for map
    lifecycle_manager_map = Node(
	    package='nav2_lifecycle_manager',
	    executable='lifecycle_manager',
	    name='lifecycle_manager_localization',
	    output='screen',
	    parameters=[{
	        'use_sim_time': True,
	        'autostart': True,
	        'node_names': ['map_server']
	    }]
    )


	

    default_nav2_yaml='/root/ros_ws/src/tb3_multi_nav/config/nav2_waffle.yaml'
    # Navigation2 per robot1
    nav2_robot1 =TimerAction(
	period=2.0, #(Test to outrun sync problem) wait 3 sec for clock, doesn't seem to help   
	actions=[ GroupAction([
		        #PushRosNamespace('robot1'),  #BAD BEHAVIOR
		        IncludeLaunchDescription(
		            PythonLaunchDescriptionSource(
		                os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
		            ),
		            launch_arguments={
		                'use_sim_time': 'true', 
						'namespace':'robot1',   #WORKS BETTER
						'use_namespace':'true', 
						'autostart':'true',     #configures lifecycle nodes inside bringup
		                'map': os.path.join(get_package_share_directory('nav2_bringup'), 'maps', 'turtlebot3_world.yaml'),
		                #'params_file': os.path.join(get_package_share_directory('nav2_bringup'), 'params', 'nav2_params.yaml') #file nav2 multirobot, 1st robot
		            	'params_file': nav2_waffle_yaml1
					}.items()
		        )
		    ])
		]
    )

    nav2_robot2 =TimerAction(
	period=2.0, #(Test to outrun sync problem) wait 3 sec for clock, doesn't seem to help  
	actions=[ GroupAction([
	        #PushRosNamespace('robot2'), #BAD BEHAVIOR
	        IncludeLaunchDescription(
	            PythonLaunchDescriptionSource(
	                os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
	            ),
	            launch_arguments={
	                'use_sim_time': 'true',
					'namespace':'robot2',
					'use_namespace':'true',
					'autostart':'true',		#configures lifecycle nodes inside bringup
	                'map': os.path.join(get_package_share_directory('nav2_bringup'), 'maps', 'turtlebot3_world.yaml'),
	                #'params_file': os.path.join(get_package_share_directory('nav2_bringup'), 'params', 'nav2_params.yaml') #file multirobot nav2, 2nd robot
					'params_file': nav2_waffle_yaml2 #file multirobot nav2, 2nd robot

				}.items()
	        )
	    ])]
    )
	# Robot1 initial pose
    init_pose_robot1 = TimerAction(
	period=2.0, #(Test to outrun sync problem) wait 3 sec for clock, doesn't seem to help   
	actions=[
		ExecuteProcess(
			cmd=[
				'ros2', 'topic', 'pub', '--once',
				'/robot1/initialpose',
				'geometry_msgs/PoseWithCovarianceStamped',
				'{header: {frame_id: "map"}, pose: {pose: {position: {x: -0.5, y: 0.0, z: 0.01}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}}'
			],
			output='screen')]
	)

	# Robot2 initial pose
    init_pose_robot2 = TimerAction(
		period=2.0, #(Test to outrun sync problem) wait 3 sec for clock, doesn't seem to help   
		actions=[
				ExecuteProcess(
					cmd=[
						'ros2', 'topic', 'pub', '--once',
						'/robot2/initialpose',
						'geometry_msgs/PoseWithCovarianceStamped',
						'{header: {frame_id: "map"}, pose: {pose: {position: {x: -0.5, y: 1.0, z: 0.01}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}}'
					],
					output='screen')]
	)
	
	#To sync main map topic with 2 robots
    map_republisher=Node(
			package="tb3_multi_nav",
			executable="map_republisher",
			name="map_republisher",
			namespace="",
			output="screen"
	)
    # RViz2 configuration
    rviz_config = os.path.join(get_package_share_directory('nav2_bringup'), 'rviz', 'nav2_default_view.rviz')
    '''
    rviz_config = os.path.join(
        get_package_share_directory('tb3_multi_nav'),
        'config', 'multi_nav.rviz'
    )
    '''
    rviz_node1 =TimerAction(
	period=2.0, #wait 2 sec for clock 
	actions=[
			Node(
		        package='rviz2',
		        executable='rviz2',
		        arguments=['-d', rviz_config],
		        output='screen',
		        parameters=[{'use_sim_time':True}]
    		)]
    )
    cloned_robots=ExecuteProcess(
			cmd=[
				'ros2', 'launch',  'nav2_bringup',
				'cloned_multi_tb3_simulation_launch.py',
				'robots:="{robot1: {x: -0.5, y: 0.0, z: 0.01, yaw: 0.0}, robot2: {x: -0.5, y: 0.0, z: 0.01, yaw: 0.0}}"'
			],
			output='screen')

    # "Tasks" (nodes) executed when using launch file
    return LaunchDescription([
        gazebo,
        spawn_robot1,
        spawn_robot2,
		bridge_robot1,
		bridge_robot2,
		map_server,
		map_server1,  		#test qui
		map_server2,
		lifecycle_manager_map,
		#map_republisher,    #test republisher map
		robot1_state_publisher,
		robot2_state_publisher,
		#cloned_robots,
		#map_to_odom_robot1,
		#map_to_odom_robot2,
		#odom_to_base_robot1,
		#odom_to_base_robot2,
		#broadcaster_odom_robot1,
		#broadcaster_odom_robot2,
        nav2_robot1,
        nav2_robot2,
		init_pose_robot1,
		init_pose_robot2,
        rviz_node1
    ])