from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    """Launch the composable action server in a container."""
    
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

    return LaunchDescription([container])
