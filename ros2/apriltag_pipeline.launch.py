# M7 — GPU AprilTag pipeline (runs in the isaac container).
#
# isaac_ros_apriltag's AprilTagNode (CUDA backend, NITROS zero-copy) detects
# tag36h11 markers on the bridged Gazebo camera and publishes
# /tag_detections (AprilTagDetectionArray) with the full 3D pose of each tag
# in the camera optical frame — this is what the M5 pipeline had to estimate
# by hand from pixel offsets + altitude.
#
#   ros2 launch /lab/ros2/apriltag_pipeline.launch.py
#
# Remaps: image        <- /camera/image        (ros_gz_bridge)
#         camera_info  <- /camera/camera_info
# size = 0.60 m: tag edge on the spawned pad (sim/models/apriltag_36h11_0).

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    apriltag_node = ComposableNode(
        package='isaac_ros_apriltag',
        plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
        name='apriltag',
        remappings=[
            ('image', '/camera/image'),
            ('camera_info', '/camera/camera_info'),
        ],
        parameters=[{'size': 0.60,
                     'max_tags': 4,
                     'tile_size': 4}])

    container = ComposableNodeContainer(
        package='rclcpp_components',
        name='apriltag_container',
        namespace='',
        executable='component_container_mt',
        composable_node_descriptions=[apriltag_node],
        output='screen')

    return launch.LaunchDescription([container])
