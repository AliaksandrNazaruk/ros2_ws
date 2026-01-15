#!/usr/bin/env python3
"""
Launch action to check if a node exists before launching
This prevents duplicate nodes from being started
"""

from launch import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import subprocess
import os


def check_node_action(context, node_name):
    """Check if node exists and return appropriate action"""
    from launch.actions import LogInfo, Shutdown
    
    script_path = os.path.join(
        os.path.dirname(__file__),
        '..',
        'scripts',
        'check_node_exists.py'
    )
    
    try:
        result = subprocess.run(
            ['python3', script_path, node_name],
            capture_output=True,
            timeout=3.0
        )
        
        if result.returncode == 0:
            # Node exists - log warning and shutdown launch
            return [
                LogInfo(msg=f'⚠️  Node {node_name} already exists! Skipping launch to prevent duplicate.'),
                Shutdown(reason=f'Node {node_name} already running')
            ]
        else:
            # Node doesn't exist - continue
            return []
    except Exception as e:
        # On error, continue (don't block launch)
        return [LogInfo(msg=f'Could not check for node {node_name}: {e}. Continuing...')]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('node_name', default_value=''),
        OpaqueFunction(
            function=lambda context: check_node_action(
                context,
                context.launch_configurations['node_name']
            )
        )
    ])
