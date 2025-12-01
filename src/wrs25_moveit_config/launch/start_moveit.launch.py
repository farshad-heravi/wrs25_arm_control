import os
from launch import LaunchDescription
from launch_param_builder import get_package_share_directory
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction, IncludeLaunchDescription, OpaqueFunction, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    declared_arguments = [
        DeclareLaunchArgument(
            'real_robot',
            default_value='true',
            description='Use real robot',
        ),
        DeclareLaunchArgument(
            'fake_robotiq_gripper',
            default_value='false',
            description='Use fake Robotiq gripper',
        ),
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.168.1.101',
            description='Robot IP address',
        ),
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='false',
            description='Use fake hardware',
        ),
        DeclareLaunchArgument(
            'fake_sensor_commands',
            default_value='false',
            description='Use fake sensor commands',
        ),
        DeclareLaunchArgument(
            'com_port',
            default_value='/dev/ttyUSB0',
            description='gripper\'s COM port',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Use rviz',
        ),
    ]

    def launch_setup(context, *args, **kwargs):
        use_fake_hardware = context.launch_configurations.get('use_fake_hardware', False)
        real_robot = context.launch_configurations.get('real_robot', False)
        use_sim_time = context.launch_configurations.get('use_sim_time', False)
        use_rviz = context.launch_configurations.get('use_rviz', True)
        robot_ip = context.launch_configurations.get('robot_ip', '192.168.1.101')
        com_port = context.launch_configurations.get('com_port', '/dev/ttyUSB0')
        fake_sensor_commands = context.launch_configurations.get('fake_sensor_commands', False)
        fake_robotiq_gripper = context.launch_configurations.get('fake_robotiq_gripper', False)
        real_robot_bool = real_robot=='true'
        use_sim_time_bool = use_sim_time=='true'
        
        if real_robot_bool:
            moveit_config_file = "config/moveit_controllers_real_robot.yaml"
            joint_controllers_file = os.path.join( get_package_share_directory('wrs25_moveit_config'), 'config', 'ros2_controllers.yaml' )
            log1 = LogInfo( msg=['Loading Controllers for Real Robot'] )
        else:
            moveit_config_file = "config/moveit_controllers_simulation.yaml"
            joint_controllers_file = os.path.join( get_package_share_directory('wrs25_moveit_config'), 'config', 'ros2_controllers.yaml' )
            log1 = LogInfo( msg=['Loading Controllers for Simulation'] )

        rviz_config = PathJoinSubstitution(
            [FindPackageShare("wrs25_moveit_config"), "config", "moveit.rviz"]
        )

        # moveit config
        moveit_config = (
            MoveItConfigsBuilder("wrs_cell_v2", package_name="wrs25_moveit_config")
            .robot_description(file_path="config/wrs_cell_v2.urdf.xacro",
                mappings={
                    "use_fake_hardware": use_fake_hardware,
                    "fake_sensor_commands": fake_sensor_commands,
                    "robot_ip": robot_ip,
                    "com_port": com_port,
                    "use_sim_time": use_sim_time,
                    "use_rviz": use_rviz,
                })
            .robot_description_semantic(file_path="config/wrs_cell_v2.srdf")
            .trajectory_execution(file_path=moveit_config_file)
            .robot_description_kinematics(file_path="config/kinematics.yaml")
            .joint_limits(file_path="config/joint_limits.yaml")
            .planning_scene_monitor(
                publish_robot_description= True, publish_robot_description_semantic=True, publish_planning_scene=True
            )
            .planning_pipelines(
                pipelines=["chomp", "ompl", "pilz_industrial_motion_planner"],
                default_planning_pipeline="pilz_industrial_motion_planner"
            )
            .to_moveit_configs()
        )

        # Robot State Publisher
        robot_state_publisher_node = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[moveit_config.robot_description, {'use_sim_time': use_sim_time_bool}],
            output="screen"
        )

        # include ur_robot_bringup.py
        ur_robot_bringup_node = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(get_package_share_directory('wrs25_moveit_config'), 'launch', 'ur_robot_bringup.py')
                    ),
                    launch_arguments={
                        'ur_type': 'ur5e',
                        'robot_ip': robot_ip,
                        'launch_rviz': 'false',
                        'robot_state_pub_node': 'false',
                    }.items(),
                    condition=IfCondition(real_robot),
                )

        # Robot State Publisher - Merger node
        robot_joint_state_merger_node = Node(
            package="wrs25_moveit_config",
            executable="joint_state_merger.py",
            output="screen",
            parameters=[{
                'fake_joint_state': LaunchConfiguration('fake_robotiq_gripper'),
            }],
            # condition=IfCondition(real_robot)
        )

        # start robotiq_gripper_action_server to publish joint state for real/fake gripper
        robotiq_gripper_action_server_node = Node(
            package="wrs25_arm_actions",
            executable="robotiq_gripper_action_server.py",
            output="screen",
            parameters=[{
                'fake_joint_state': LaunchConfiguration('fake_robotiq_gripper'),
            }]
        )

        # Controller Manager - only for simulation (fake hardware)
        controller_manager_node = Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                moveit_config.robot_description,
                joint_controllers_file,
                {'use_sim_time': use_sim_time_bool},
            ],
            remappings=[('/joint_states', '/ur_internal/joint_states')],
            output="screen",
            condition=IfCondition(use_fake_hardware)
        )

        # Spawner for real robot controller - starts scaled_joint_trajectory_controller from UR driver
        # The UR driver loads this controller but doesn't start it by default
        scaled_joint_trajectory_controller_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["scaled_joint_trajectory_controller", "--controller-manager", "/controller_manager"],
            output="screen",
            condition=IfCondition(real_robot)
        )

        # Spawners for simulation controllers - only when using fake hardware
        ur5_arm_controller_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["ur5_arm_controller", "--controller-manager", "/controller_manager"],
            output="screen",
            condition=IfCondition(use_fake_hardware)
        )

        # Spawner for hand controller - only for simulation (fake hardware)
        hand_controller_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["hand_controller", "--controller-manager", "/controller_manager", "--param-file", joint_controllers_file],
            output="screen",
        )
        delayed_hand_spawner = TimerAction(
            period=4.0,
            actions=[hand_controller_spawner]
        )

        # Spawner for joint state broadcaster - only for simulation (fake hardware)
        joint_state_broadcaster_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen",
            remappings=[('/joint_states', '/ur_internal/joint_states')],
            condition=IfCondition(use_fake_hardware)
        )

        # move group node
        move_group_node = Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                {'use_sim_time': use_sim_time_bool},
                {"trajectory_execution.allowed_start_tolerance": 0.05},
                {"trajectory_execution.allowed_goal_duration_margin": 0.5},
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

        timer_action = TimerAction(
            period=8.0,
            actions=[move_group_node, rviz_node]
        )



        return [
            log1,
            ur_robot_bringup_node,
            robot_state_publisher_node,
            robot_joint_state_merger_node,
            robotiq_gripper_action_server_node,
            controller_manager_node,
            scaled_joint_trajectory_controller_spawner, 
            ur5_arm_controller_spawner,
            delayed_hand_spawner,
            joint_state_broadcaster_spawner,
            timer_action,
        ]

    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=launch_setup)
    ])