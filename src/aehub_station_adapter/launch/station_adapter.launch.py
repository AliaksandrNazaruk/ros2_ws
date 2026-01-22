from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    symovo_endpoint_arg = DeclareLaunchArgument(
        "symovo_endpoint",
        default_value=EnvironmentVariable("SYMOVO_ENDPOINT", default_value="https://192.168.1.100"),
        description="Symovo API endpoint (base URL).",
    )
    tls_verify_arg = DeclareLaunchArgument(
        "tls_verify",
        default_value="false",
        description="Verify TLS certificates for Symovo HTTPS (true/false).",
    )
    http_timeout_arg = DeclareLaunchArgument("http_timeout_sec", default_value="5.0")

    # Note: station_adapter is implemented as a LifecycleNode, but it auto-configures and
    # auto-activates internally on startup. Launch it as a normal process node.
    node = Node(
        package="aehub_station_adapter",
        executable="station_adapter",
        name="station_adapter",
        namespace="",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"symovo_endpoint": LaunchConfiguration("symovo_endpoint")},
            {"tls_verify": LaunchConfiguration("tls_verify")},
            {"http_timeout_sec": LaunchConfiguration("http_timeout_sec")},
        ],
    )

    return LaunchDescription([symovo_endpoint_arg, tls_verify_arg, http_timeout_arg, node])

