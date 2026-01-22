import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("aehub_nav2_gazebo")
    nav2_share = get_package_share_directory("nav2_bringup")
    gazebo_share = get_package_share_directory("gazebo_ros")

    default_world = os.path.join(pkg_share, "worlds", "open_space.world")
    default_map = os.path.join(pkg_share, "maps", "open_space.yaml")
    default_params = os.path.join(pkg_share, "params", "nav2_params.yaml")
    default_urdf = os.path.join(pkg_share, "urdf", "amr.urdf")

    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    world = LaunchConfiguration("world")
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    launch_navigation_integrated_node = LaunchConfiguration("launch_navigation_integrated_node")

    robot_id = LaunchConfiguration("robot_id")
    config_service_url = LaunchConfiguration("config_service_url")
    config_service_api_key = LaunchConfiguration("config_service_api_key")

    with open(default_urdf, "r", encoding="utf-8") as urdf_file:
        robot_description = urdf_file.read()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_share, "launch", "gazebo.launch.py")),
        launch_arguments={
            "world": world,
            "gui": gui,
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "robot_description": robot_description}],
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "aehub_amr", "-x", "0.0", "-y", "0.0", "-z", "0.1"],
        output="screen",
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_share, "launch", "bringup_launch.py")),
        launch_arguments={
            "map": map_yaml,
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "autostart": autostart,
        }.items(),
    )

    navigation_integrated = Node(
        package="aehub_navigation",
        executable="navigation_integrated_node",
        name="navigation_integrated_node",
        output="screen",
        condition=IfCondition(launch_navigation_integrated_node),
        parameters=[
            {
                "robot_id": robot_id,
                "config_service_url": config_service_url,
                "config_service_api_key": config_service_api_key,
                "cmd_vel_topic": "cmd_vel",
                "status_publish_hz": 2.0,
                "command_process_hz": 20.0,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("map", default_value=default_map),
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument(
                "launch_navigation_integrated_node",
                default_value="true",
                description="Launch legacy navigation_integrated_node (MQTT gateway). Disable when using clean MQTT stack.",
            ),
            DeclareLaunchArgument(
                "robot_id",
                default_value=EnvironmentVariable("ROBOT_ID", default_value="fahrdummy-01-local_sim"),
            ),
            DeclareLaunchArgument(
                "config_service_url",
                default_value=EnvironmentVariable("CONFIG_SERVICE_URL", default_value="http://localhost:7900"),
            ),
            DeclareLaunchArgument(
                "config_service_api_key",
                default_value=EnvironmentVariable("CONFIG_SERVICE_API_KEY", default_value=""),
            ),
            gazebo,
            robot_state_publisher,
            joint_state_publisher,
            spawn_entity,
            nav2_bringup,
            navigation_integrated,
        ]
    )
