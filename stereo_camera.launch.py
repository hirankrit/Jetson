#!/usr/bin/env python3
"""
Launch file for IMX219 Stereo Camera on Jetson
Uses GStreamer nvarguscamerasrc backend
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for stereo camera node"""

    # Declare launch arguments
    left_sensor_id_arg = DeclareLaunchArgument(
        'left_sensor_id',
        default_value='0',
        description='Sensor ID for left camera'
    )

    right_sensor_id_arg = DeclareLaunchArgument(
        'right_sensor_id',
        default_value='1',
        description='Sensor ID for right camera'
    )

    width_arg = DeclareLaunchArgument(
        'width',
        default_value='1280',
        description='Image width in pixels'
    )

    height_arg = DeclareLaunchArgument(
        'height',
        default_value='720',
        description='Image height in pixels'
    )

    framerate_arg = DeclareLaunchArgument(
        'framerate',
        default_value='30',
        description='Camera framerate (fps)'
    )

    flip_method_arg = DeclareLaunchArgument(
        'flip_method',
        default_value='0',
        description='Image flip method (0=none, 2=rotate-180)'
    )

    # Create stereo camera node
    stereo_camera_node = Node(
        package='pepper_vision',  # Will be created in ROS2 workspace
        executable='stereo_camera_node',
        name='stereo_camera_node',
        output='screen',
        parameters=[{
            'left_sensor_id': LaunchConfiguration('left_sensor_id'),
            'right_sensor_id': LaunchConfiguration('right_sensor_id'),
            'width': LaunchConfiguration('width'),
            'height': LaunchConfiguration('height'),
            'framerate': LaunchConfiguration('framerate'),
            'flip_method': LaunchConfiguration('flip_method'),
        }],
        emulate_tty=True,
    )

    return LaunchDescription([
        left_sensor_id_arg,
        right_sensor_id_arg,
        width_arg,
        height_arg,
        framerate_arg,
        flip_method_arg,
        stereo_camera_node,
    ])
