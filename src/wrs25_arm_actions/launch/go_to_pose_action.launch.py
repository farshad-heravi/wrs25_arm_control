from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    """Launch the composable action server in a container."""
    
    # Declare launch arguments for Robotiq gripper
    default_ip_arg = DeclareLaunchArgument(
        'robotiq_ip',
        default_value='192.168.1.101',
        description='Default IP address of the Robotiq gripper'
    )
    
    default_port_arg = DeclareLaunchArgument(
        'robotiq_port',
        default_value='63352',
        description='Default port of the Robotiq gripper'
    )
    
    auto_connect_arg = DeclareLaunchArgument(
        'robotiq_auto_connect',
        default_value='false',
        description='Automatically connect to Robotiq gripper on startup'
    )
    
    # Get path to kinematics configuration
    moveit_config_pkg = get_package_share_directory('wrs_env_v2_moveit_config')
    kinematics_yaml_path = os.path.join(moveit_config_pkg, 'config', 'kinematics.yaml')
    
    # Load kinematics parameters from YAML file
    with open(kinematics_yaml_path, 'r') as file:
        kinematics_config = yaml.safe_load(file)
    
    # Prepare parameters with proper namespace
    robot_description_kinematics = {'robot_description_kinematics': kinematics_config}
    
    server_component = ComposableNode(
        package='wrs25_arm_actions',
        plugin='wrs25_arm_actions::GoToPoseActionServer',
        name='go_to_pose_action_server',
        parameters=[robot_description_kinematics],
    )

    gripper_server_component = ComposableNode(
        package='wrs25_arm_actions',
        plugin='wrs25_arm_actions::GripperControlActionServer',
        name='gripper_control_action_server',
    )

    container = ComposableNodeContainer(
        name='wrs_action_server_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[server_component, gripper_server_component],
        output='screen',
    )

    # Robotiq gripper action server (Python node, separate from C++ container)
    robotiq_gripper_server = Node(
        package='wrs25_arm_actions',
        executable='robotiq_gripper_action_server.py',
        name='robotiq_gripper_action_server',
        output='screen',
        parameters=[{
            'default_ip': LaunchConfiguration('robotiq_ip'),
            'default_port': LaunchConfiguration('robotiq_port'),
            'auto_connect': LaunchConfiguration('robotiq_auto_connect'),
        }]
    )

    return LaunchDescription([
        default_ip_arg,
        default_port_arg,
        auto_connect_arg,
        container,
        robotiq_gripper_server
    ])
