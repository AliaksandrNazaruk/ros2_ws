#!/usr/bin/env python3

# Copyright 2026 Boris
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

"""
Launch file for Symovo AMR with Nav2 integration.

Launches base_controller, Symovo bridge (scan), Nav2 navigation stack, and
navigation_integrated_node (MQTT bridge).

IMPORTANT: Map is automatically loaded from Symovo API using load_symovo_map.py
which correctly accounts for offsetX and offsetY. If map_file is provided,
it will be used instead of auto-loading.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
import sys
import fcntl


# Global stack lock to prevent duplicate launch instances.
# We keep the file handle open for the lifetime of the launch process.
_STACK_LOCK_FILE = None
_STACK_LOCK_PATH = "/tmp/symovo_nav2_stack.lock"


def acquire_stack_lock(context):
    """Prevent launching duplicate Symovo Nav2 stacks."""
    global _STACK_LOCK_FILE  # noqa: PLW0603 (global is intentional for lock lifetime)
    try:
        fh = open(_STACK_LOCK_PATH, "w")
        try:
            fcntl.flock(fh.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            msg = (
                "CRITICAL ERROR: symovo_nav2 stack is already running!\n"
                "Another launch instance holds the stack lock.\n"
                "Stop the running stack before starting a new one.\n"
                f"Lock file: {_STACK_LOCK_PATH}"
            )
            print(msg, file=sys.stderr)
            try:
                fh.close()
            except Exception:
                pass
            raise RuntimeError(msg)

        # Lock acquired: write PID and keep file open
        fh.write(str(os.getpid()) + "\n")
        fh.flush()
        _STACK_LOCK_FILE = fh
        return [LogInfo(msg=f"✅ Stack lock acquired: {_STACK_LOCK_PATH}")]
    except Exception as e:
        raise RuntimeError(f"Failed to acquire stack lock {_STACK_LOCK_PATH}: {e}")


def load_map_from_symovo(context):
    """Load map from Symovo API using aehub_navigation.symovo_map_loader (handles offsetX/offsetY)."""
    provided_map_file = context.launch_configurations.get('map_file', '')

    if provided_map_file and provided_map_file != '':
        if os.path.exists(provided_map_file):
            # Keep resolved_map_file for other nodes / debugging
            context.launch_configurations['resolved_map_file'] = provided_map_file
            return [LogInfo(msg=f'✅ Using provided map file: {provided_map_file}')]
        return [LogInfo(msg=f'❌ Provided map file not found: {provided_map_file}')]

    symovo_endpoint = context.launch_configurations.get('symovo_endpoint', 'https://192.168.1.100')
    amr_id = context.launch_configurations.get('amr_id', '15')
    map_output_dir = context.launch_configurations.get('map_output_dir', '')

    if not map_output_dir:
        return [LogInfo(msg='❌ map_output_dir is empty (unexpected).')]

    os.makedirs(map_output_dir, exist_ok=True)

    try:
        from aehub_navigation.symovo_map_loader import SymovoMapLoader
        loader = SymovoMapLoader(symovo_endpoint, int(amr_id), tls_verify=False)
        yaml_path = loader.load_map(map_output_dir)
        if yaml_path and os.path.exists(yaml_path):
            context.launch_configurations['resolved_map_file'] = yaml_path
            return [LogInfo(msg=f'✅ Map loaded successfully from Symovo API: {yaml_path}')]

        # Fallback: use previously downloaded map if present
        fallback = os.path.join(map_output_dir, 'map.yaml')
        if os.path.exists(fallback):
            context.launch_configurations['resolved_map_file'] = fallback
            return [LogInfo(msg=f'⚠️  Using existing map.yaml (API fetch failed): {fallback}')]

        return [LogInfo(msg='⚠️  Failed to load map from Symovo API (no yaml produced and no fallback)')]
    except Exception as e:
        return [LogInfo(msg=f'⚠️  Error loading map from Symovo API: {e}')]


def generate_launch_description():
    """Generate ROS2 launch description for the full Symovo+Nav2 stack."""
    # Launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    workspace_root = LaunchConfiguration('workspace_root')

    # Declare launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_workspace_root = DeclareLaunchArgument(
        'workspace_root',
        default_value=os.environ.get('ROS2_WS', '/home/boris/ros2_ws'),
        description='Workspace root path (used to locate config/ and maps/ directories)'
    )

    # Default params file path (<workspace_root>/config/nav2_symovo_params.yaml)
    default_params_file = PythonExpression(["'", workspace_root, "/config/nav2_symovo_params.yaml'"])

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file'
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='log level'
    )

    # Symovo bridge args
    declare_use_scan_converter = DeclareLaunchArgument(
        'use_scan_converter',
        default_value='true',
        description='Enable Symovo scan converter (PNG scan -> LaserScan)'
    )

    # TF hygiene: ensure scan frame can be transformed to base frame.
    # Without base_link->laser static TF, AMCL / costmaps drop /scan and map->odom never appears,
    # causing lifecycle_manager_navigation to abort bringup (bt_navigator stays inactive and rejects goals).
    declare_publish_base_laser_tf = DeclareLaunchArgument(
        'publish_base_laser_tf',
        default_value='true',
        description='Publish static TF base_frame_id->laser_frame_id via tf2_ros/static_transform_publisher'
    )
    declare_base_frame_id = DeclareLaunchArgument(
        'base_frame_id',
        default_value='base_link',
        description='Base frame id for static TF (parent)'
    )
    declare_laser_frame_id = DeclareLaunchArgument(
        'laser_frame_id',
        default_value='laser',
        description='Laser frame id for static TF (child); must match scan frame_id'
    )
    declare_base_to_laser_x = DeclareLaunchArgument('base_to_laser_x', default_value='0.0')
    declare_base_to_laser_y = DeclareLaunchArgument('base_to_laser_y', default_value='0.0')
    declare_base_to_laser_z = DeclareLaunchArgument('base_to_laser_z', default_value='0.0')
    # RPY (radians) for static TF publisher (x y z yaw pitch roll parent child)
    declare_base_to_laser_yaw = DeclareLaunchArgument('base_to_laser_yaw', default_value='0.0')
    declare_base_to_laser_pitch = DeclareLaunchArgument('base_to_laser_pitch', default_value='0.0')
    declare_base_to_laser_roll = DeclareLaunchArgument('base_to_laser_roll', default_value='0.0')

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

    declare_tls_verify = DeclareLaunchArgument(
        'tls_verify',
        default_value='false',
        description='Verify TLS certificates when calling Symovo API (false for self-signed certs)'
    )

    declare_symovo_auto_enable_drive_mode = DeclareLaunchArgument(
        'symovo_auto_enable_drive_mode',
        default_value='false',
        description='Try to enable Symovo drive mode automatically before navigation (best-effort)'
    )

    declare_symovo_ready_check_enabled = DeclareLaunchArgument(
        'symovo_ready_check_enabled',
        default_value='true',
        description='Check Symovo drive_ready state before accepting navigation commands'
    )

    declare_symovo_ready_check_fail_open = DeclareLaunchArgument(
        'symovo_ready_check_fail_open',
        default_value='false',
        description='If true, ignore Symovo readiness check failures and proceed with navigation (for testing)'
    )

    # MQTT bridge args
    declare_robot_id = DeclareLaunchArgument(
        'robot_id',
        default_value='robot_001',
        description='Robot ID for MQTT topics'
    )

    declare_config_service_url = DeclareLaunchArgument(
        'config_service_url',
        default_value='http://localhost:7900',
        description='URL of Broker Config Service'
    )

    declare_config_service_api_key = DeclareLaunchArgument(
        'config_service_api_key',
        default_value='',
        description='API key for Broker Config Service (REQUIRED)'
    )

    declare_config_poll_interval = DeclareLaunchArgument(
        'config_poll_interval',
        default_value='5.0',
        description='Interval for polling Config Service for changes (seconds)'
    )

    # Default positions file in package share
    try:
        pkg_aehub_navigation = get_package_share_directory('aehub_navigation')
        default_positions_file = os.path.join(pkg_aehub_navigation, 'config', 'positions.yaml')
    except Exception:
        default_positions_file = ''

    declare_positions_file = DeclareLaunchArgument(
        'positions_file',
        default_value=default_positions_file,
        description='Path to positions YAML file'
    )

    # Map loading configuration
    default_map_output_dir = PythonExpression(["'", workspace_root, "/maps/symovo_map'"])

    declare_map_output_dir = DeclareLaunchArgument(
        'map_output_dir',
        default_value=default_map_output_dir,
        description='Directory where map will be loaded from Symovo API'
    )

    declare_map_file = DeclareLaunchArgument(
        'map_file',
        default_value='',
        description='Path to map YAML (if empty, auto-load from Symovo API with offsetX/offsetY)'
    )

    base_controller_node = Node(
        package='base_controller',
        executable='base_controller_node',
        name='base_controller',
        namespace=namespace,
        # Pass minimal required params explicitly to avoid relying on missing YAML
        parameters=[{
            'driver_endpoint': LaunchConfiguration('symovo_endpoint'),
            'amr_id': LaunchConfiguration('amr_id'),
            'tls_verify': LaunchConfiguration('tls_verify'),
        }],
        output='screen'
    )

    # Nav2 bringup group
    nav2_bringup_group = GroupAction([
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': autostart},
                # Costmaps / BT bringup can be slow on real hardware; avoid premature aborts
                {'bond_timeout': 10.0},
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

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file]
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file]
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file]
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='recoveries_server',
            output='screen',
            parameters=[params_file]
        ),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[params_file]
        ),

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

    ])

    # Symovo bridge launch (scan converter)
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
            'tls_verify': LaunchConfiguration('tls_verify'),
        }.items()
    )

    # Map loading action (runs before map_server)
    load_map_action = OpaqueFunction(function=load_map_from_symovo)

    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'bond_timeout': 10.0},
            {'node_names': ['map_server', 'amcl']}
        ]
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            # Must be a STRING parameter (not string_array)
            'yaml_filename': PythonExpression([
                "'", LaunchConfiguration('map_file'), "' if '", LaunchConfiguration('map_file'), "' != '' else '",
                default_map_output_dir, "/map.yaml'"
            ])
        }]
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file]
    )

    navigation_integrated_node = Node(
        package='aehub_navigation',
        executable='navigation_integrated_node.py',
        name='navigation_integrated_node',
        output='screen',
        parameters=[{
            'robot_id': LaunchConfiguration('robot_id'),
            'config_service_url': LaunchConfiguration('config_service_url'),
            'config_service_api_key': LaunchConfiguration('config_service_api_key'),
            'config_poll_interval': LaunchConfiguration('config_poll_interval'),
            'positions_file': LaunchConfiguration('positions_file'),
            # Pass Symovo connection parameters so the node can reject navigation in manual mode
            'symovo_endpoint': LaunchConfiguration('symovo_endpoint'),
            'amr_id': LaunchConfiguration('amr_id'),
            'tls_verify': LaunchConfiguration('tls_verify'),
            'symovo_auto_enable_drive_mode': LaunchConfiguration('symovo_auto_enable_drive_mode'),
            'symovo_ready_check_enabled': LaunchConfiguration('symovo_ready_check_enabled'),
            'symovo_ready_check_fail_open': LaunchConfiguration('symovo_ready_check_fail_open'),
        }]
    )

    # Publish initialpose automatically based on Symovo API pose to bootstrap AMCL (map->odom)
    initial_pose_from_symovo = Node(
        package='aehub_navigation',
        executable='initial_pose_from_symovo.py',
        name='initial_pose_from_symovo',
        output='screen',
        parameters=[{
            'symovo_endpoint': LaunchConfiguration('symovo_endpoint'),
            'amr_id': LaunchConfiguration('amr_id'),
            'tls_verify': LaunchConfiguration('tls_verify'),
        }]
    )

    static_tf_base_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_laser',
        output='screen',
        condition=IfCondition(LaunchConfiguration('publish_base_laser_tf')),
        arguments=[
            LaunchConfiguration('base_to_laser_x'),
            LaunchConfiguration('base_to_laser_y'),
            LaunchConfiguration('base_to_laser_z'),
            LaunchConfiguration('base_to_laser_yaw'),
            LaunchConfiguration('base_to_laser_pitch'),
            LaunchConfiguration('base_to_laser_roll'),
            LaunchConfiguration('base_frame_id'),
            LaunchConfiguration('laser_frame_id'),
        ]
    )

    ld = LaunchDescription()
    # Must be first: prevent duplicate launch instances creating duplicate nodes.
    ld.add_action(OpaqueFunction(function=acquire_stack_lock))
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_workspace_root)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_use_scan_converter)
    ld.add_action(declare_publish_base_laser_tf)
    ld.add_action(declare_base_frame_id)
    ld.add_action(declare_laser_frame_id)
    ld.add_action(declare_base_to_laser_x)
    ld.add_action(declare_base_to_laser_y)
    ld.add_action(declare_base_to_laser_z)
    ld.add_action(declare_base_to_laser_yaw)
    ld.add_action(declare_base_to_laser_pitch)
    ld.add_action(declare_base_to_laser_roll)
    ld.add_action(declare_symovo_endpoint)
    ld.add_action(declare_amr_id)
    ld.add_action(declare_tls_verify)
    ld.add_action(declare_symovo_auto_enable_drive_mode)
    ld.add_action(declare_symovo_ready_check_enabled)
    ld.add_action(declare_symovo_ready_check_fail_open)
    ld.add_action(declare_map_output_dir)
    ld.add_action(declare_map_file)
    ld.add_action(declare_robot_id)
    ld.add_action(declare_config_service_url)
    ld.add_action(declare_config_service_api_key)
    ld.add_action(declare_config_poll_interval)
    ld.add_action(declare_positions_file)

    ld.add_action(load_map_action)

    ld.add_action(base_controller_node)
    ld.add_action(static_tf_base_laser)
    ld.add_action(lifecycle_manager_localization)
    ld.add_action(map_server_node)
    ld.add_action(amcl_node)
    ld.add_action(initial_pose_from_symovo)
    ld.add_action(symovo_bridge_launch)

    # Delay navigation bringup to allow:
    # - map_server + amcl to become ACTIVE
    # - initial_pose_from_symovo to publish /initialpose
    # This prevents nav lifecycle activation failing due to missing map->odom TF.
    ld.add_action(TimerAction(period=8.0, actions=[nav2_bringup_group, navigation_integrated_node]))

    return ld

