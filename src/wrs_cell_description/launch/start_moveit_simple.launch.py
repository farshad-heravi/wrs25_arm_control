from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time',
        ),
    ]

    # Get MoveIt Config
    moveit_config = (
        MoveItConfigsBuilder("wrs_cell_v2", package_name="wrs_env_v2_moveit_config")
        .to_moveit_configs()
    )

    # RViz config file
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("wrs_env_v2_moveit_config"), "config", "moveit.rviz"]
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    # Controller Manager Node
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": moveit_config.robot_description.get("robot_description")},
            PathJoinSubstitution([FindPackageShare("wrs_env_v2_moveit_config"), "config", "ros2_controllers.yaml"]),
        ],
        output="screen",
    )

    # Spawners for each controller
    ur5_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ur5_arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["hand_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Move Group Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {"publish_robot_description_semantic": True},
        ],
    )

    # RViz Node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    # Group MoveIt nodes with a TimerAction
    moveit_nodes_timer = TimerAction(
        period=4.0,  # Increased delay to 5 seconds
        actions=[GroupAction(
            actions=[
                move_group_node,
                rviz_node,
            ],
        )]
    )

    return LaunchDescription(declared_arguments + [
        robot_state_publisher_node,
        controller_manager_node,
        ur5_arm_controller_spawner,
        hand_controller_spawner,
        joint_state_broadcaster_spawner,
        moveit_nodes_timer,
    ])
