    lifecycle_manager_robot1 = Node(
		package='nav2_lifecycle_manager',
		executable='lifecycle_manager',
		name='lifecycle_manager_robot1',
		output='screen',
		namespace='robot1',
		parameters=[{
			'use_sim_time': True,
			'autostart': True,
			'node_names': ['amcl','planner_server','controller_server','bt_navigator','smoother_server']
		}]
    )
    lifecycle_manager_robot2 = Node(
		package='nav2_lifecycle_manager',
		executable='lifecycle_manager',
		name='lifecycle_manager_robot2',
		output='screen',
		namespace='robot2',
		parameters=[{
			'use_sim_time': True,
			'autostart': True,
			'node_names': ['amcl','planner_server','controller_server','bt_navigator','smoother_server']
		}]
    )
        '''
    # Map server for Rviz2 and Nav2 --Robot1
    map_server_robot1 = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='robot1',
        output='screen',
        parameters=[{
            'yaml_filename': '/root/ros_ws/install/nav2_bringup/share/nav2_bringup/maps/turtlebot3_world.yaml',
            'use_sim_time': True
        }]
    )

    # Map server for Rviz2 and Nav2 --Robot2
    map_server_robot2 = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='robot2',
        output='screen',
        parameters=[{
            'yaml_filename': '/root/ros_ws/install/nav2_bringup/share/nav2_bringup/maps/turtlebot3_world.yaml',
            'use_sim_time': True
        }]
    )
    '''
        amcl_robot1=TimerAction(
		period=2.0,  #wait 2 sec for clock
		actions=[
				GroupAction([
					PushRosNamespace('robot1'),
					LifecycleNode(
						package='nav2_amcl',
						executable='amcl',
						name='amcl',
						namespace='',
						output='screen',
						parameters=[nav2_waffle_yaml1]
					)
				])
			]
	)
    amcl_robot2=TimerAction(
			period=2.0,  #wait 2 sec for clock
			actions=[
					GroupAction([
						PushRosNamespace('robot2'),
						LifecycleNode(
						package='nav2_amcl',
						executable='amcl',
						name='amcl',
						namespace='',
						output='screen',
						parameters=[nav2_waffle_yaml2]
						)
					])
			]
		)


    planner_robot1 = TimerAction(
			period=2.0,  #wait 2 sec for clock
			actions=[
				GroupAction([
				PushRosNamespace('robot1'),
				LifecycleNode(
					package='nav2_planner',
					executable='planner_server',
					name='planner_server',
					namespace='',
					output='screen',
					parameters=[nav2_waffle_yaml1]
				)
			])
	])

    controller_robot1 = TimerAction(
			period=2.0,  #wait 2 sec for clock
			actions=[
				GroupAction([
				PushRosNamespace('robot1'),
				LifecycleNode(
					package='nav2_controller',
					executable='controller_server',
					name='controller_server',
					namespace='',
					output='screen',
					parameters=[nav2_waffle_yaml1]
				)
			])
	])

    bt_navigator_robot1 = TimerAction(
			period=2.0,  #wait 2 sec for clock
			actions=[
				GroupAction([
				PushRosNamespace('robot1'),
				LifecycleNode(
					package='nav2_bt_navigator',
					executable='bt_navigator',
					name='bt_navigator',
					namespace='',
					output='screen',
					parameters=[nav2_waffle_yaml1]
				)
				])
			]
	)
			

    smoother_robot1 = TimerAction(
			period=2.0,  #wait 2 sec for clock
			actions=[
				GroupAction([
				PushRosNamespace('robot1'),
				LifecycleNode(
					package='nav2_smoother',
					executable='smoother_server',
					name='smoother_server',
					namespace='',
					output='screen',
					parameters=[nav2_waffle_yaml1]
				)])
			]
	)

    planner_robot2 = TimerAction(
			period=2.0,  #wait 2 sec for clock
			actions=[ 
				GroupAction([
				PushRosNamespace('robot2'),
				LifecycleNode(
					package='nav2_planner',
					executable='planner_server',
					name='planner_server',
					namespace='',
					output='screen',
					parameters=[nav2_waffle_yaml2]
				)
				])
			]
	)
    

    controller_robot2 =TimerAction(
			period=2.0,  #wait 2 sec for clock
			actions=[
				GroupAction([
				PushRosNamespace('robot2'), 
				LifecycleNode(
					package='nav2_controller',
					executable='controller_server',
					name='controller_server',
					namespace='',
					output='screen',
					parameters=[nav2_waffle_yaml2]
					)
				])
			]
	)
    

    bt_navigator_robot2 =  TimerAction(
			period=2.0,  #wait 2 sec for clock
			actions=[
				GroupAction([
				PushRosNamespace('robot2'),
				LifecycleNode(
					package='nav2_bt_navigator',
					executable='bt_navigator',
					name='bt_navigator',
					namespace='',
					output='screen',
					parameters=[nav2_waffle_yaml2]
				)
				])
			]
	)
    

    smoother_robot2 =TimerAction(
			period=2.0,  #wait 2 sec for clock
			actions=[ 
					GroupAction([
					PushRosNamespace('robot2'),
					LifecycleNode(
					package='nav2_smoother',
					executable='smoother_server',
					name='smoother_server',
					namespace='',
					output='screen',
					parameters=[nav2_waffle_yaml2]
					)
				])
			]
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
	#manager for every lifecycle node. "node_names" can be extended to include all lifecycle nodes.
    lifecycle_manager = Node(
	    package='nav2_lifecycle_manager',
	    executable='lifecycle_manager',
	    name='lifecycle_manager_localization',
	    output='screen',
	    parameters=[{
	        'use_sim_time': True,
	        'autostart': True,
	        'node_names': ['map_server','amcl','amcl2','planner_server1','planner_server2',
							'controller_server1','controller_server2','bt_navigator1','bt_navigator2',
							'smoother_server1','smoother_server2']
	    }]
    )

    '''
	#to launch 2 rviz window
    rviz_node2 =TimerAction( 
	period=2.0, #wait 2 sec for clock 
	actions=[ 	PushRosNamespace("robot2"),
			Node(
		        package='rviz2',
		        executable='rviz2',
		        arguments=['-d', rviz_config],
		        output='screen',
		        parameters=[{'use_sim_time':True}]
    		)]
    )
    '''