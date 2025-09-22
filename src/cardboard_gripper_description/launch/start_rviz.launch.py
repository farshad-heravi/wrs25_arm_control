from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import PathJoinSubstitution, Command
import os
import xacro


def generate_launch_description():
    pkg_share = FindPackageShare("cardboard_gripper_description").find("cardboard_gripper_description")
    
    # Set the path to the URDF file
    urdf_file = os.path.join(pkg_share, "gripper", "gripper.urdf.xacro")
    
    # Process the XACRO file
    robot_description_content = Command([
        'xacro ',
        urdf_file
    ])
    
    # Create a ParameterValue for robot_description
    robot_description = ParameterValue(robot_description_content, value_type=str)
    
    # Define RViz config path if it exists
    rviz_config_path = os.path.join(pkg_share, "rviz", "gripper.rviz")
    if not os.path.exists(rviz_config_path):
        rviz_config_path = ""
    
    # Create and return launch description
    return LaunchDescription([
        # Publish the robot state
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        
        # Launch joint state publisher
#        Node(
#            package="joint_state_publisher",
#            executable="joint_state_publisher",
#            name="joint_state_publisher",
#            output="screen",
#        ),
        
        # Launch joint state publisher GUI for interactive joint control
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen",
        ),
        
        # Launch RViz
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config_path] if rviz_config_path else [],
        )
    ])
