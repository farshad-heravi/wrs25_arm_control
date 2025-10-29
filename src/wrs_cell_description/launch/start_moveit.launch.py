import os
from launch import LaunchDescription
from launch_param_builder import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time',
        ),
    ]

    moveit_config = (
        MoveItConfigsBuilder("wrs_cell_v2", package_name="wrs_env_v2_moveit_config")
        .robot_description(file_path="config/wrs_cell_v2.urdf.xacro")
        .robot_description_semantic(file_path="config/wrs_cell_v2.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_scene_monitor(
            publish_robot_description= True, publish_robot_description_semantic=True, publish_planning_scene=True
        )
        .planning_pipelines(
            pipelines=["chomp", "ompl", "pilz_industrial_motion_planner"],
            default_planning_pipeline="chomp"
        )
        .to_moveit_configs()
    )
    print("Planning pipeline config file:", moveit_config.planning_pipelines)

    rviz_config = PathJoinSubstitution(
        [FindPackageShare("wrs_env_v2_moveit_config"), "config", "moveit.rviz"]
    )

    joint_controllers_file = os.path.join(
        get_package_share_directory('wrs_env_v2_moveit_config'), 'config', 'ros2_controllers.yaml'
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
        ],
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            joint_controllers_file,
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

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {"trajectory_execution.allowed_start_tolerance": 0.05},
            {"trajectory_execution.allowed_goal_duration_margin": 0.5},
            # {"publish_robot_description_semantic": True},
            # {"default_planner_pipeline": "ompl"},
            # {"ompl.planning_plugin": "ompl_interface/OMPLPlanner"},
            # {"planning_plugin": "ompl_interface/OMPLPlanner"},
            # {"ompl.default_planner_config": "RRTConnect"},
            {"log_level": "DEBUG"},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[moveit_config.to_dict()],
    )

    moveit_nodes_timer = TimerAction(
        period=8.0,
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
