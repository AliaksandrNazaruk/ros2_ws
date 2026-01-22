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
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetLaunchConfiguration,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch_ros.actions import LifecycleNode, Node, PushRosNamespace
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory
from lifecycle_msgs.msg import Transition
import os
import sys
import fcntl

# region agent log
def _agent_log(*, run_id: str, hypothesis_id: str, location: str, message: str, data: dict):
    try:
        import json
        import time

        os.makedirs("/home/boris/ros2_ws/.cursor", exist_ok=True)
        with open("/home/boris/ros2_ws/.cursor/debug.log", "a", encoding="utf-8") as f:
            f.write(
                json.dumps(
                    {
                        "sessionId": "debug-session",
                        "runId": run_id,
                        "hypothesisId": hypothesis_id,
                        "location": location,
                        "message": message,
                        "data": data,
                        "timestamp": int(time.time() * 1000),
                    },
                    ensure_ascii=False,
                )
                + "\n"
            )
    except Exception:
        pass


# endregion agent log


def _autostart_lifecycle(node: LifecycleNode, *, configure_delay_s: float, activate_delay_s: float):
    """Helper to auto-configure and activate a LifecycleNode."""
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda action: action == node,
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )
    activate_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda action: action == node,
            transition_id=Transition.TRANSITION_ACTIVATE,
        )
    )

    configure_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=node,
            on_start=[TimerAction(period=configure_delay_s, actions=[configure_event])],
        )
    )
    # IMPORTANT: rclpy LifecycleNode will throw if we request ACTIVATE from UNCONFIGURED.
    # Chain activation on successful CONFIGURE (i.e., when the node reaches INACTIVE).
    activate_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=node,
            goal_state="inactive",
            entities=[TimerAction(period=activate_delay_s, actions=[activate_event])],
        )
    )

    return [configure_handler, activate_handler]


def _namespace_parts_from_launch_arg(context) -> list[str]:
    ns = LaunchConfiguration("namespace").perform(context) or ""
    ns = str(ns).strip().strip("/")
    if not ns:
        return []
    raw_parts = [p for p in ns.split("/") if p]
    parts: list[str] = []
    for p in raw_parts:
        # ROS name segments must not start with a digit; sanitize to keep launch robust.
        seg = "".join(ch if (ch.isalnum() or ch == "_") else "_" for ch in str(p))
        if seg and seg[0].isdigit():
            seg = f"r{seg}"
        if seg:
            parts.append(seg)
    return parts


def _push_namespace_from_launch_arg(context):
    """
    Build PushRosNamespace actions from 'namespace' launch argument.

    Accepts:
    - '' (no namespace)
    - 'robot/15' or '/robot/15' (multi-segment)
    - 'robot' (single segment)

    Returns a list of PushRosNamespace actions, one per segment.
    """
    return [PushRosNamespace(p) for p in _namespace_parts_from_launch_arg(context)]


def _set_namespace_key(context):
    """
    Compute a fully-qualified namespace key for parameter files.

    Example: namespace:=robot/15 -> '/robot/r15'
    """
    parts = _namespace_parts_from_launch_arg(context)
    ns_key = "/" + "/".join(parts) if parts else ""
    return [SetLaunchConfiguration("namespace_key", ns_key)]



def _log_on_process_start(*, target_action, label: str):
    """
    Debug-mode helper: write a debug.log entry when a process actually starts.
    """
    return RegisterEventHandler(
        OnProcessStart(
            target_action=target_action,
            on_start=[
                OpaqueFunction(
                    function=lambda context: _agent_log(
                        run_id=os.environ.get("AEHUB_DEBUG_RUN_ID", "launch"),
                        hypothesis_id="H_START",
                        location="symovo_nav2.launch.py:OnProcessStart",
                        message="process_started",
                        data={"label": label},
                    )
                    or []
                )
            ],
        )
    )


def _log_on_process_exit(*, target_action, label: str):
    """
    Debug-mode helper: write a debug.log entry when a process exits.
    """
    return RegisterEventHandler(
        OnProcessExit(
            target_action=target_action,
            on_exit=[
                OpaqueFunction(
                    function=lambda context: _agent_log(
                        run_id=os.environ.get("AEHUB_DEBUG_RUN_ID", "launch"),
                        hypothesis_id="H_EXIT",
                        location="symovo_nav2.launch.py:OnProcessExit",
                        message="process_exited",
                        data={"label": label},
                    )
                    or []
                )
            ],
        )
    )

# Stack lock to prevent duplicate launch instances for the same robot.
# We keep the file handle open for the lifetime of the launch process.
_STACK_LOCK_FILE = None


def acquire_stack_lock(context):
    """Prevent launching duplicate Symovo Nav2 stacks."""
    global _STACK_LOCK_FILE  # noqa: PLW0603 (global is intentional for lock lifetime)
    robot_id = LaunchConfiguration("robot_id").perform(context) or "unknown_robot"
    lock_path = f"/tmp/symovo_nav2_stack.{robot_id}.lock"
    # region agent log
    _agent_log(
        run_id="pre-fix",
        hypothesis_id="H1",
        location="symovo_nav2.launch.py:acquire_stack_lock",
        message="Acquire lock (and check symovo_map_fetcher executable presence)",
        data={"robot_id": robot_id, "lock_path": lock_path},
    )
    try:
        from ament_index_python.packages import get_package_prefix

        prefix = get_package_prefix("aehub_navigation")
        expected_exec = os.path.join(prefix, "lib", "aehub_navigation", "symovo_map_fetcher")
        _agent_log(
            run_id="pre-fix",
            hypothesis_id="H2",
            location="symovo_nav2.launch.py:acquire_stack_lock",
            message="Computed aehub_navigation prefix and expected symovo_map_fetcher path",
            data={
                "prefix": prefix,
                "expected_exec": expected_exec,
                "exists": os.path.exists(expected_exec),
            },
        )
    except Exception as e:
        _agent_log(
            run_id="pre-fix",
            hypothesis_id="H2",
            location="symovo_nav2.launch.py:acquire_stack_lock",
            message="Failed to compute prefix/expected executable",
            data={"error": str(e)},
        )
    # endregion agent log
    try:
        fh = open(lock_path, "w")
        try:
            fcntl.flock(fh.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            msg = (
                "CRITICAL ERROR: symovo_nav2 stack is already running!\n"
                "Another launch instance holds the stack lock.\n"
                "Stop the running stack before starting a new one.\n"
                f"Lock file: {lock_path}"
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
        return [LogInfo(msg=f"✅ Stack lock acquired: {lock_path}")]
    except Exception as e:
        raise RuntimeError(f"Failed to acquire stack lock {lock_path}: {e}")


def generate_launch_description():
    """Generate ROS2 launch description for the full Symovo+Nav2 stack."""
    # Launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    workspace_root = LaunchConfiguration('workspace_root')
    launch_base_controller = LaunchConfiguration('launch_base_controller')
    launch_navigation_integrated_node = LaunchConfiguration('launch_navigation_integrated_node')

    # Declare launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description="Top-level namespace ('' or 'robot/15' or '/robot/15')."
    )

    # Nav2 best-practice: rewrite parameters under the applied namespace.
    nav2_params_file = RewrittenYaml(
        source_file=params_file,
        root_key=LaunchConfiguration("namespace_key"),
        param_rewrites={},
        convert_types=True,
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

    # Default params file:
    # Prefer package share config (installed), fallback to workspace_root/config for dev.
    try:
        pkg_share = get_package_share_directory('aehub_navigation')
        default_params_path = os.path.join(pkg_share, 'config', 'nav2_symovo_params.yaml')
    except Exception:
        default_params_path = ''

    if not default_params_path or not os.path.exists(default_params_path):
        default_params_file = PythonExpression(["'", workspace_root, "/config/nav2_symovo_params.yaml'"])
    else:
        default_params_file = default_params_path

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

    # Nav2 bringup delay (real HW can be slow; avoid premature Nav2 activation before map->odom exists)
    declare_nav2_bringup_delay = DeclareLaunchArgument(
        'nav2_bringup_delay_s',
        default_value='15.0',
        description='Delay (seconds) before starting Nav2 navigation bringup.',
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

    declare_publish_base_footprint_tf = DeclareLaunchArgument(
        'publish_base_footprint_tf',
        default_value='false',
        description='Publish static TF base_link->base_footprint (use only if not provided by URDF)'
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

    declare_tls_verify = DeclareLaunchArgument(
        'tls_verify',
        default_value='false',
        description='Verify TLS certificates when calling Symovo API (false for self-signed certs)'
    )

    declare_launch_base_controller = DeclareLaunchArgument(
        'launch_base_controller',
        default_value='true',
        description='Launch base_controller_node from this launch file (set false when provided by external bringup)',
    )

    declare_launch_navigation_integrated_node = DeclareLaunchArgument(
        'launch_navigation_integrated_node',
        default_value='true',
        description='Launch legacy navigation_integrated_node MQTT gateway (set false when using clean MQTT transport stack)',
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
        condition=IfCondition(LaunchConfiguration('launch_base_controller')),
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
            parameters=[nav2_params_file]
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params_file]
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_params_file]
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='recoveries_server',
            output='screen',
            parameters=[nav2_params_file]
        ),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[nav2_params_file]
        ),

        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[nav2_params_file],
            remappings=[
                ('cmd_vel_in', 'cmd_vel'),
                ('cmd_vel_out', 'cmd_vel'),
            ]
        ),

    ])

    # Symovo bridge launch (scan converter) - optional dependency.
    # Some deployments do not have symovo_bridge installed; map/odom integration should still run.
    symovo_bridge_launch = None
    try:
        _ = get_package_share_directory('symovo_bridge')
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
    except Exception:
        symovo_bridge_launch = LogInfo(
            msg="⚠️  symovo_bridge not installed; skipping scan converter bringup"
        )

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
                LaunchConfiguration('map_output_dir'), "/map.yaml'"
            ]),
        }]
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params_file]
    )

    navigation_integrated_node = Node(
        package='aehub_navigation',
        executable='navigation_integrated_node',
        name='navigation_integrated_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_navigation_integrated_node')),
        parameters=[{
            # Force string type even if user passes a numeric launch arg.
            'robot_id': ParameterValue(LaunchConfiguration('robot_id'), value_type=str),
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
        executable='initial_pose_from_symovo',
        name='initial_pose_from_symovo',
        output='screen',
        parameters=[{
            'symovo_endpoint': LaunchConfiguration('symovo_endpoint'),
            'amr_id': LaunchConfiguration('amr_id'),
            'tls_verify': LaunchConfiguration('tls_verify'),
        }]
    )

    # -------------------------------------------------------------------------
    # NEW map sync flow (SRS-compliant): map mirror + localization orchestrator
    # -------------------------------------------------------------------------
    declare_use_legacy_map_fetcher = DeclareLaunchArgument(
        'use_legacy_map_fetcher',
        default_value='false',
        description='Use legacy one-shot symovo_map_fetcher from aehub_navigation (fallback only)'
    )

    symovo_map_mirror = LifecycleNode(
        package='aehub_symovo_map_mirror',
        executable='symovo_map_mirror',
        name='symovo_map_mirror',
        namespace='',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('use_legacy_map_fetcher')),
        parameters=[{
            'symovo_endpoint': LaunchConfiguration('symovo_endpoint'),
            'amr_id': LaunchConfiguration('amr_id'),
            'tls_verify': LaunchConfiguration('tls_verify'),
            'map_output_dir': LaunchConfiguration('map_output_dir'),
            # Runtime contract defaults (tunable):
            'map_select_mode': 'pose_map_id',
            'origin_mode': 'origin_offsets',
            'pose_transform_mode': 'pose_subtract_offsets_flip_y',
            'update_mode': 'poll',
            'poll_period_sec': 2.0,
            'map_status_topic': 'infra/map/status',
            'write_absolute_image_path': False,
        }]
    )

    nav2_localization_orchestrator = LifecycleNode(
        package='aehub_nav2_localization_orchestrator',
        executable='nav2_localization_orchestrator',
        name='nav2_localization_orchestrator',
        namespace='',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('use_legacy_map_fetcher')),
        parameters=[{
            'map_status_topic': 'infra/map/status',
            'symovo_endpoint': LaunchConfiguration('symovo_endpoint'),
            'amr_id': LaunchConfiguration('amr_id'),
            'tls_verify': LaunchConfiguration('tls_verify'),
            'map_server_load_map_service': 'map_server/load_map',
            'initialpose_topic': 'initialpose',
        }]
    )

    # Auto-start lifecycle for map sync nodes (when new flow is active)
    map_sync_handlers = []
    # RPi / real HW can be slow to import Python modules; keep these delays conservative
    # so ChangeState doesn't fire before the lifecycle services are ready.
    map_sync_handlers += _autostart_lifecycle(symovo_map_mirror, configure_delay_s=2.5, activate_delay_s=1.0)
    map_sync_handlers += _autostart_lifecycle(nav2_localization_orchestrator, configure_delay_s=3.5, activate_delay_s=1.5)
    # Debug evidence: confirm these processes actually start/exit.
    map_sync_handlers += [_log_on_process_start(target_action=symovo_map_mirror, label="symovo_map_mirror")]
    map_sync_handlers += [_log_on_process_start(target_action=nav2_localization_orchestrator, label="nav2_localization_orchestrator")]
    map_sync_handlers += [_log_on_process_exit(target_action=symovo_map_mirror, label="symovo_map_mirror")]
    map_sync_handlers += [_log_on_process_exit(target_action=nav2_localization_orchestrator, label="nav2_localization_orchestrator")]

    # Legacy one-shot fetcher (writes map.yaml + map.pgm, then exits).
    # Kept only as a migration fallback behind a feature flag.
    symovo_map_fetcher = Node(
        package='aehub_navigation',
        executable='symovo_map_fetcher',
        name='symovo_map_fetcher',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_legacy_map_fetcher')),
        parameters=[{
            'symovo_endpoint': LaunchConfiguration('symovo_endpoint'),
            'amr_id': LaunchConfiguration('amr_id'),
            'tls_verify': LaunchConfiguration('tls_verify'),
            'output_dir': LaunchConfiguration('map_output_dir'),
            'select_map_mode': 'robot_pose',
            'write_absolute_image_path': False,
        }]
    )


    # --- Static TF: base_link -> base_footprint
    base_to_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_footprint',
        condition=IfCondition(LaunchConfiguration('publish_base_footprint_tf')),
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
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
    ld.add_action(OpaqueFunction(function=_set_namespace_key))
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_workspace_root)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_nav2_bringup_delay)
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
    ld.add_action(declare_publish_base_footprint_tf)
    ld.add_action(declare_symovo_endpoint)
    ld.add_action(declare_amr_id)
    ld.add_action(declare_tls_verify)
    ld.add_action(declare_launch_base_controller)
    ld.add_action(declare_launch_navigation_integrated_node)
    ld.add_action(declare_symovo_auto_enable_drive_mode)
    ld.add_action(declare_symovo_ready_check_enabled)
    ld.add_action(declare_symovo_ready_check_fail_open)
    ld.add_action(declare_map_output_dir)
    ld.add_action(declare_map_file)
    ld.add_action(declare_use_legacy_map_fetcher)
    ld.add_action(declare_robot_id)
    ld.add_action(declare_config_service_url)
    ld.add_action(declare_config_service_api_key)
    ld.add_action(declare_config_poll_interval)
    ld.add_action(declare_positions_file)

    # Namespaced robot stack (SRS: /robot/<id>/... namespace support).
    robot_stack = GroupAction([
        OpaqueFunction(function=_push_namespace_from_launch_arg),
        base_controller_node,
        base_to_footprint,
        static_tf_base_laser,
        symovo_bridge_launch,
        # If map_file is provided, start localization immediately (no Symovo map sync).
        GroupAction(
            condition=UnlessCondition(PythonExpression(["'", LaunchConfiguration('map_file'), "' == ''"])),
            actions=[
                lifecycle_manager_localization,
                map_server_node,
                amcl_node,
                # For explicit map_file we still keep legacy initial pose helper for now.
                initial_pose_from_symovo,
            ],
        ),
        # If map_file is empty:
        # - new flow: map mirror runs continuously + orchestrator reloads map_server and publishes initialpose
        # - legacy flow: one-shot fetcher gates initial startup
        *map_sync_handlers,  # Auto-configure/activate lifecycle nodes (must be registered before node start)
        # New flow: once map mirror is ACTIVE, start localization nodes (map_server+amcl)
        # and let the orchestrator reload map_server via LoadMap and publish /initialpose.
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=symovo_map_mirror,
                goal_state="active",
                entities=[
                    TimerAction(
                        period=1.0,
                        actions=[
                            # NOTE: event-triggered actions may run outside the current namespace context.
                            # Explicitly re-push namespace from launch arg to keep localization nodes namespaced.
                            GroupAction(
                                actions=[
                                    OpaqueFunction(function=_push_namespace_from_launch_arg),
                                    lifecycle_manager_localization,
                                    map_server_node,
                                    amcl_node,
                                ]
                            ),
                        ],
                    )
                ],
            ),
            condition=IfCondition(
                PythonExpression([
                    "'", LaunchConfiguration('map_file'), "' == '' and '",
                    LaunchConfiguration('use_legacy_map_fetcher'), "' == 'false'"
                ])
            ),
        ),
        symovo_map_mirror,
        nav2_localization_orchestrator,
        symovo_map_fetcher,
        RegisterEventHandler(
            OnProcessExit(
                target_action=symovo_map_fetcher,
                on_exit=[
                    lifecycle_manager_localization,
                    map_server_node,
                    amcl_node,
                    initial_pose_from_symovo,
                ],
            ),
            condition=IfCondition(
                PythonExpression([
                    "'", LaunchConfiguration('map_file'), "' == '' and '",
                    LaunchConfiguration('use_legacy_map_fetcher'), "' == 'true'"
                ])
            ),
        ),
        # Delay navigation bringup to allow localization to become ACTIVE and initial pose to be published.
        TimerAction(period=LaunchConfiguration('nav2_bringup_delay_s'), actions=[nav2_bringup_group, navigation_integrated_node]),
    ])

    ld.add_action(robot_stack)

    return ld

