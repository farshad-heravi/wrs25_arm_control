from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction
from launch.actions import GroupAction
import os

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time',
        ),
        DeclareLaunchArgument(
            'doCalibration',
            default_value='false',
            description='Force calibration',
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
            'use_real_robot',
            default_value='true',
            description='Use real robot',
        ),
    ]
    ur_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('wrs25_moveit_config'), 'launch', 'start_moveit.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_real_robot': LaunchConfiguration('use_real_robot'),
            'robot_ip': LaunchConfiguration('robot_ip'),
            'use_fake_hardware': LaunchConfiguration('use_fake_hardware'),
            'fake_sensor_commands': LaunchConfiguration('fake_sensor_commands'),
        }.items(),
    )

    # bringup camera nodes
    camera_nodes_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('cam_calibration'), 'launch', 'bringup_camera_nodes.launch.py')
        ),
        launch_arguments={
            'doCalibration': LaunchConfiguration('doCalibration'),
        }.items(),
    )

    # bringup arm action servers
    action_server_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('wrs25_arm_actions'), 'launch', 'bringup_actions.launch.py')
        ),
        launch_arguments={
            'robotiq_ip': LaunchConfiguration('robotiq_ip'),
            'robotiq_port': LaunchConfiguration('robotiq_port'),
            'robotiq_auto_connect': LaunchConfiguration('robotiq_auto_connect'),
        }.items(),
    )
    action_server_bringup_launch_timer = TimerAction(
        period = 6.0,
        actions=[action_server_bringup_launch],
    )

    # bring up vision action server
    vision_action_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('wrs25_pose_estimation_module_ros2'), 'launch', 'bringup_vision_action_servers.launch.py')
        ),
    )

    return LaunchDescription([
        ur_bringup_launch,
        camera_nodes_launch,
        vision_action_server_launch,
        action_server_bringup_launch_timer,
    ])