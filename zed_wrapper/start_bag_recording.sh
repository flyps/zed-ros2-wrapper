#!/bin/bash

MAX_BAG_DUR=300 # Max .bag file length in seconds

source /ros2_ws/install/setup.bash

# Launch the ROS2 node
ros2 launch zed_wrapper stream_topics.launch.py camera_config:="zedx_dataset_collector.yaml" &

cd /ros2_ws/bags && ros2 bag record /zed/zed_node/left/camera_info /zed/zed_node/left/image_rect_color -d ${MAX_BAG_DUR}