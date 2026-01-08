#!/usr/bin/env python3

"""
Launch file for Symovo AMR with Nav2 integration.
Launches base_controller, Nav2 navigation stack, and navigation_integrated_node (MQTT bridge).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get the launch directory
    pkg_nav2_bringup = FindPackageShare(package='nav2_bringup').find('nav2_bringup')
    pkg_base_controller = FindPackageShare(package='base_controller').find('base_controller')
    
    # Launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    
    # Declare launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    # Default params file path (in workspace config directory)
    default_params_file = os.path.join(
        os.path.dirname(os.path.dirname(__file__)),
        'config',
        'nav2_symovo_params.yaml'
    )
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file to use for all launched nodes')
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack')
    
    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='True',
        description='Whether to use composed bringup')
    
    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name',
        default_value='nav2_container',
        description='the name of conatiner that nodes will load in if use composition')
    
    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')
    
    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='log level')
    
    # Symovo bridge arguments
    declare_use_scan_converter = DeclareLaunchArgument(
        'use_scan_converter',
        default_value='true',
        description='Enable Symovo scan converter (converts PNG scan to LaserScan)'
    )
    
    declare_symovo_endpoint = DeclareLaunchArgument(
        'symovo_endpoint',
        default_value='https://192.168.1.100',
        description='Symovo API endpoint'
    )
    
    declare_amr_id = DeclareLaunchArgument(
        'amr_id',
        default_value='15',
        description='Symovo AMR ID'
    )
    
    # Navigation integrated node arguments
    declare_robot_id = DeclareLaunchArgument(
        'robot_id',
        default_value='robot_001',
        description='Robot ID for MQTT topics'
    )
    
    declare_config_service_url = DeclareLaunchArgument(
        'config_service_url',
        default_value='http://localhost:7900',
        description='URL of Broker Config Service (centralized MQTT configuration)'
    )
    
    declare_config_service_api_key = DeclareLaunchArgument(
        'config_service_api_key',
        default_value='',
        description='API key for Broker Config Service authentication (REQUIRED for navigation_integrated_node)'
    )
    
    declare_config_poll_interval = DeclareLaunchArgument(
        'config_poll_interval',
        default_value='5.0',
        description='Interval for polling Config Service for changes (seconds)'
    )
    
    # Get default positions file path
    try:
        pkg_aehub_navigation = get_package_share_directory('aehub_navigation')
        default_positions_file = os.path.join(pkg_aehub_navigation, 'config', 'positions.yaml')
    except:
        default_positions_file = ''
    
    declare_positions_file = DeclareLaunchArgument(
        'positions_file',
        default_value=default_positions_file,
        description='Path to positions YAML file (optional, used by navigation_integrated_node)'
    )
    
    # Base Controller config path (optional - node will use defaults if not found)
    base_controller_config_path = os.path.join(
        os.path.dirname(os.path.dirname(__file__)),
        'config',
        'base_controller.yaml'
    )
    
    # Base Controller Node
    base_controller_params = []
    if os.path.exists(base_controller_config_path):
        base_controller_params = [base_controller_config_path]
    
    base_controller_node = Node(
        package='base_controller',
        executable='base_controller_node',
        name='base_controller',
        namespace=namespace,
        parameters=base_controller_params,
        output='screen'
    )
    
    # Nav2 bringup group
    nav2_bringup_group = GroupAction([
        # Nav2 lifecycle manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': autostart},
                {'node_names': [
                    'controller_server',
                    'planner_server',
                    'recoveries_server',
                    'bt_navigator',
                    'waypoint_follower',
                    'velocity_smoother',
                ]}
            ]
        ),
        
        # Controller server - publishes to /cmd_vel by default
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file]
        ),
        
        # Planner server
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file]
        ),
        
        # BT Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file]
        ),
        
        # Recoveries server
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='recoveries_server',
            output='screen',
            parameters=[params_file]
        ),
        
        # Waypoint follower
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[params_file]
        ),
        
        # Velocity smoother (optional - can be disabled if not needed)
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[params_file],
            remappings=[
                ('/cmd_vel_in', '/cmd_vel'),
                ('/cmd_vel_out', '/cmd_vel'),
            ]
        ),
        
        # Costmap nodes
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='local_costmap',
            output='screen',
            parameters=[params_file]
        ),
        
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='global_costmap',
            output='screen',
            parameters=[params_file]
        ),
    ])
    
    # Symovo bridge launch (scan converter and map loader)
    symovo_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('symovo_bridge'),
                'launch',
                'symovo_bridge.launch.py'
            ])
        ]),
        launch_arguments={
            'symovo_endpoint': LaunchConfiguration('symovo_endpoint'),
            'amr_id': LaunchConfiguration('amr_id'),
            'use_scan_converter': LaunchConfiguration('use_scan_converter'),
        }.items()
    )
    
    # Map server node
    # NOTE: Map MUST be loaded once at startup via yaml_filename parameter
    # NO runtime map switching (AE.HUB MVP requirement)
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': LaunchConfiguration('map_file')  # Static map file path
        }]
    )
    
    # Declare map_file argument
    declare_map_file = DeclareLaunchArgument(
        'map_file',
        default_value='',
        description='Path to static map YAML file (required for AE.HUB MVP)'
    )
    
    # Navigation integrated node (MQTT bridge for Nav2)
    # NOTE: This node handles MQTT commands and sends goals to Nav2
    # It requires Config Service API key to fetch MQTT broker configuration
    navigation_integrated_node = Node(
        package='aehub_navigation',
        executable='navigation_integrated_node',
        name='navigation_integrated_node',
        output='screen',
        parameters=[{
            'robot_id': LaunchConfiguration('robot_id'),
            'config_service_url': LaunchConfiguration('config_service_url'),
            'config_service_api_key': LaunchConfiguration('config_service_api_key'),
            'config_poll_interval': LaunchConfiguration('config_poll_interval'),
            'positions_file': LaunchConfiguration('positions_file'),
        }]
    )
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_use_scan_converter)
    ld.add_action(declare_symovo_endpoint)
    ld.add_action(declare_amr_id)
    ld.add_action(declare_map_file)
    ld.add_action(declare_robot_id)
    ld.add_action(declare_config_service_url)
    ld.add_action(declare_config_service_api_key)
    ld.add_action(declare_config_poll_interval)
    ld.add_action(declare_positions_file)
    
    # Add the nodes
    ld.add_action(base_controller_node)
    ld.add_action(map_server_node)  # Map server must be started before map_loader
    ld.add_action(symovo_bridge_launch)  # Scan converter and map loader
    ld.add_action(nav2_bringup_group)
    ld.add_action(navigation_integrated_node)  # MQTT bridge for Nav2 (must start after Nav2)
    
    return ld

