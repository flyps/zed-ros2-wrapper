from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

from launch_ros.actions import Node


def generate_launch_description():
    # start the ZED X
    zed_wrapper_path = FindPackageShare(package='zed_wrapper').find('zed_wrapper')
    start_zedx = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(zed_wrapper_path, "launch", "zed_camera.launch.py")),
        launch_arguments={
            'camera_model': 'zedx',
            'ros_params_override_path': os.path.join(zed_wrapper_path, "config",
                                                     "zedx_streamer.yaml")
        }.items()
    )

    # start web video server
    # ros2 run web_video_server web_video_server
    start_web_video_server = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        output='screen'
    )

    return LaunchDescription([
        start_zedx,
        start_web_video_server
    ])
