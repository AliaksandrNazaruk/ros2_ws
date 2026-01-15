#!/usr/bin/env python3

"""
Launch file for Symovo bridge nodes (scan converter and map loader)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Launch configuration variables
    symovo_endpoint = LaunchConfiguration('symovo_endpoint')
    amr_id = LaunchConfiguration('amr_id')
    tls_insecure = LaunchConfiguration('tls_insecure')
    tls_verify = LaunchConfiguration('tls_verify')
    use_scan_converter = LaunchConfiguration('use_scan_converter')
    use_map_loader = LaunchConfiguration('use_map_loader')
    params_file = LaunchConfiguration('params_file')
    
    # Declare launch arguments
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
    
    declare_tls_insecure = DeclareLaunchArgument(
        'tls_insecure',
        default_value='true',
        description='Disable TLS certificate validation'
    )

    declare_tls_verify = DeclareLaunchArgument(
        'tls_verify',
        default_value='false',
        description='Verify TLS certificate (overrides tls_insecure when explicitly set)'
    )
    
    declare_use_scan_converter = DeclareLaunchArgument(
        'use_scan_converter',
        default_value='true',
        description='Enable scan converter node'
    )
    
    # Default params file path
    default_params_file = os.path.join(
        FindPackageShare('symovo_bridge').find('symovo_bridge'),
        'config',
        'symovo_bridge.yaml'
    )
    
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to parameters file'
    )
    
    # Scan converter node
    scan_converter_node = Node(
        package='symovo_bridge',
        executable='symovo_scan_converter.py',
        name='symovo_scan_converter',
        # params_file provides defaults; explicit overrides allow launch-time control
        parameters=[
            params_file,
            {
                'symovo_endpoint': symovo_endpoint,
                'amr_id': amr_id,
                'tls_verify': tls_verify,
            }
        ],
        condition=IfCondition(use_scan_converter),
        output='screen'
    )
    
    ld = LaunchDescription()
    
    # Add arguments
    ld.add_action(declare_symovo_endpoint)
    ld.add_action(declare_amr_id)
    ld.add_action(declare_tls_insecure)
    ld.add_action(declare_tls_verify)
    ld.add_action(declare_use_scan_converter)
    ld.add_action(declare_params_file)
    
    # Add nodes
    ld.add_action(scan_converter_node)
    
    return ld

