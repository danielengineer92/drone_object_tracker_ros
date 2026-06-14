"""
Launch file for vision mode.

Launches real camera, YOLO detection, tracker, control, fake telemetry, and visualizer.
Requires a camera but no drone connection.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate the vision mode launch description."""
    # Get config file path
    bringup_dir = get_package_share_directory('drone_bringup')
    config_file = os.path.join(bringup_dir, 'config', 'vision_params.yaml')

    # Launch arguments
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='true',
        description='Run without display window. Use headless:=false only when a desktop display is available.'
    )

    camera_index_arg = DeclareLaunchArgument(
        'camera_index',
        default_value='0',
        description='Camera device index'
    )

    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='yolov8n.pt',
        description='Path to YOLO model file'
    )

    target_class_arg = DeclareLaunchArgument(
        'target_class',
        default_value='person',
        description='Target class to track'
    )

    # Nodes
    camera = Node(
        package='drone_camera',
        executable='camera_node',
        name='camera_node',
        parameters=[config_file, {'camera_index': LaunchConfiguration('camera_index')}],
        output='screen',
    )

    yolo = Node(
        package='drone_yolo',
        executable='yolo_node',
        name='yolo_node',
        parameters=[
            config_file,
            {
                'model_path': LaunchConfiguration('model_path'),
                'target_class': LaunchConfiguration('target_class'),
            },
        ],
        output='screen',
    )

    tracker = Node(
        package='drone_tracker',
        executable='tracker_node',
        name='tracker_node',
        parameters=[config_file, {'target_class': LaunchConfiguration('target_class')}],
        output='screen',
    )

    control = Node(
        package='drone_control',
        executable='control_node',
        name='control_node',
        parameters=[config_file],
        output='screen',
        emulate_tty=True,
    )

    # Use fake telemetry since no drone is connected
    fake_telemetry = Node(
        package='drone_fake',
        executable='fake_telemetry_node',
        name='fake_telemetry_node',
        parameters=[config_file],
        output='screen',
    )

    visualizer = Node(
        package='drone_visualizer',
        executable='visualizer_node',
        name='visualizer_node',
        parameters=[config_file, {'headless': LaunchConfiguration('headless')}],
        output='screen',
    )

    return LaunchDescription([
        headless_arg,
        camera_index_arg,
        model_path_arg,
        target_class_arg,
        LogInfo(msg='=== DRONE VISION SYSTEM - VISION MODE ==='),
        LogInfo(msg='Real camera + YOLO detection active. No drone connection.'),
        LogInfo(msg='autonomous_enabled is FALSE - no flight commands will be sent.'),
        camera,
        yolo,
        tracker,
        control,
        fake_telemetry,
        visualizer,
    ])