from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_param_builder import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'rgb_image_size',
            default_value='1280x720x30',
            description='RGB camera image size',
        ),
        DeclareLaunchArgument(
            'depth',
            default_value='false',
            description='Depth camera',
        ),
        DeclareLaunchArgument(
            'doCalibration',
            default_value='false',
            description='Force calibration',
        ),
    ]

    realsens_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
        ),
        launch_arguments=[{   'rgb_camera.profile': LaunchConfiguration('rgb_image_size'),
                        'depth': LaunchConfiguration('depth') }],
    )

    # launch calibration node
    calibration_node = Node(
        package='cam_calibration',
        executable='calibration_v2.py',
        name='calibration_node',
        parameters=[{   'force_calibration': LaunchConfiguration('doCalibration') == 'true' }],
    )

    # launch pose estimation node
    pose_estimation_node = Node(
        package='cam_calibration',
        executable='pose_estimation.py',
        name='pose_estimation_node',
    )
    
    # launch launch description
    return LaunchDescription(declared_arguments + [
        realsens_launch,
        calibration_node,
        pose_estimation_node,
    ])