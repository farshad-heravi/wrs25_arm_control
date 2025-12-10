#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch file for UR External Control Monitor Node."""
    
    # Declare launch arguments
    check_interval_arg = DeclareLaunchArgument(
        'check_interval',
        default_value='2.0',
        description='Interval in seconds between checks'
    )
    
    program_filename_arg = DeclareLaunchArgument(
        'program_filename',
        default_value='external_control.urp',
        description='Name of the external control program file'
    )
    
    dashboard_namespace_arg = DeclareLaunchArgument(
        'dashboard_namespace',
        default_value='/dashboard_client',
        description='Namespace for the dashboard services'
    )
    
    auto_start_arg = DeclareLaunchArgument(
        'auto_start',
        default_value='true',
        description='Automatically start the external control program if not running'
    )
    
    # Create the monitor node
    monitor_node = Node(
        package='wrs25_arm_actions',
        executable='ur_external_control_monitor.py',
        name='ur_external_control_monitor',
        output='screen',
        parameters=[{
            'check_interval': LaunchConfiguration('check_interval'),
            'program_filename': LaunchConfiguration('program_filename'),
            'dashboard_namespace': LaunchConfiguration('dashboard_namespace'),
            'auto_start': LaunchConfiguration('auto_start'),
        }]
    )
    
    return LaunchDescription([
        check_interval_arg,
        program_filename_arg,
        dashboard_namespace_arg,
        auto_start_arg,
        monitor_node
    ])

