#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch file for Robotiq Gripper Action Server."""
    
    # Declare launch arguments
    default_ip_arg = DeclareLaunchArgument(
        'default_ip',
        default_value='192.168.1.101',
        description='Default IP address of the Robotiq gripper'
    )
    
    default_port_arg = DeclareLaunchArgument(
        'default_port',
        default_value='63352',
        description='Default port of the Robotiq gripper'
    )
    
    auto_connect_arg = DeclareLaunchArgument(
        'auto_connect',
        default_value='false',
        description='Automatically connect to gripper on startup'
    )
    
    # Create the action server node
    robotiq_gripper_server_node = Node(
        package='wrs25_arm_actions',
        executable='robotiq_gripper_action_server.py',
        name='robotiq_gripper_action_server',
        output='screen',
        parameters=[{
            'default_ip': LaunchConfiguration('default_ip'),
            'default_port': LaunchConfiguration('default_port'),
            'auto_connect': LaunchConfiguration('auto_connect'),
        }]
    )
    
    return LaunchDescription([
        default_ip_arg,
        default_port_arg,
        auto_connect_arg,
        robotiq_gripper_server_node
    ])

