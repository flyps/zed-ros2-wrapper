from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import os

from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    camera_config = LaunchConfiguration('camera_config')
    camera_config_launch_arg = DeclareLaunchArgument(
        'camera_config',
        default_value='zedx.yaml',
        description='Path to the camera configuration file, relative to config/ directory'
    )

    # Start the ZED X node
    zed_wrapper_path = FindPackageShare(package='zed_wrapper').find('zed_wrapper')
    ros_params_override_path = PathJoinSubstitution([zed_wrapper_path, 'config', camera_config])
    start_zedx = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(zed_wrapper_path, "launch", "zed_camera.launch.py")),
        launch_arguments={
            'camera_model': 'zedx',
            'ros_params_override_path': ros_params_override_path
        }.items()
    )

    # Start web video server node
    start_web_video_server = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        output='screen'
    )

    return LaunchDescription([
        camera_config_launch_arg,  
        start_zedx,
        start_web_video_server
    ])
