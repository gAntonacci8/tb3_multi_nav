'''Buffer for future utse in mult_tb3.launch.py'''

# B. Nodo AMCL (Localizzazione)
    # ATTENZIONE: AMCL non è strettamente necessario per la SOLA mappatura SLAM
    # ma è necessario se vuoi che il robot sappia dove si trova rispetto a una mappa 
    # già costruita per la navigazione (che è ciò che fa l'esplorazione).
    amcl_node = LifecycleNode(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        parameters=[nav2_params_file, {'use_sim_time': True}],
        output='screen'
    )

    # C. Server di Pianificazione (Global Planner)
    planner_server_node = LifecycleNode(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        parameters=[nav2_params_file, {'use_sim_time': True}],
        output='screen'
    )

    # D. Server di Controllo (Local Planner)
    controller_server_node = LifecycleNode(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        parameters=[nav2_params_file, {'use_sim_time': True}],
        output='screen'
    )

    # E. Nodo Nav2 Behavior Tree (il cuore dell'esplorazione)
    # Questo nodo esegue l'albero di comportamento che implementa la logica 
    # di "vai alla frontiera più vicina" (Exploration BT).
    bt_navigator_node = LifecycleNode(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        parameters=[nav2_params_file, {'use_sim_time': True}],
        output='screen'
    )

    # F. Manager (Nodo di gestione del ciclo di vita)
    # Questo nodo si assicura che tutti i nodi Lifecycle siano avviati e attivati 
    # nel giusto ordine (inizializzazione).
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[
            {'autostart': 'true'},
            {'node_names': [
                'map_server',
                'amcl',
                'planner_server',
                'controller_server',
                'bt_navigator'
            ]}
        ]
    )