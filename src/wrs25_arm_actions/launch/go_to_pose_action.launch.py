from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """Launch the composable action server in a container."""
    
    server_component = ComposableNode(
        package='wrs25_arm_actions',
        plugin='wrs25_arm_actions::GoToPoseActionServer',
        name='go_to_pose_action_server',
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
