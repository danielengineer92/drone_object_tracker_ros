"""
Launch file for full system mode.

Launches real camera, YOLO detection, tracker, control, PX4 MAVSDK bridge,
and visualizer. Requires camera and PX4 drone connection.

WARNING: autonomy_enabled defaults to False and mavsdk_offboard_enabled defaults to False.
Commands stay IDLE until /autonomy_enable is true, and nothing is sent to PX4
until /mavsdk_offboard_enable is also true.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate the full system launch description."""

    # Get config file path
    bringup_dir = get_package_share_directory('drone_bringup')
    config_file = os.path.join(bringup_dir, 'config', 'full_system_params.yaml')

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

    connection_url_arg = DeclareLaunchArgument(
        'connection_url',
        default_value='serial:///dev/ttyACM0:57600',
        description='MAVSDK connection URL to PX4'
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

    telemetry = Node(
        package='drone_telemetry',
        executable='telemetry_node',
        name='telemetry_node',
        parameters=[config_file, {'connection_url': LaunchConfiguration('connection_url')}],
        output='screen',
    )


    autonomy_manager = Node(
        package='drone_control',
        executable='autonomy_manager_node',
        name='autonomy_manager_node',
        parameters=[config_file],
        output='screen',
        emulate_tty=True,
    )

    control = Node(
        package='drone_control',
        executable='control_node',
        name='control_node',
        parameters=[config_file],
        output='screen',
        emulate_tty=True,
    )

    visualizer = Node(
        package='drone_visualizer',
        executable='visualizer_node',
        name='visualizer_node',
        parameters=[config_file, {'headless': LaunchConfiguration('headless')}],
        output='screen',
    )

    health_monitor = Node(
        package='drone_diagnostics',
        executable='health_monitor_node',
        name='health_monitor_node',
        parameters=[config_file],
        output='screen',
    )

    return LaunchDescription([
        headless_arg,
        camera_index_arg,
        model_path_arg,
        target_class_arg,
        connection_url_arg,
        LogInfo(msg='=== DRONE VISION SYSTEM - FULL SYSTEM MODE ==='),
        LogInfo(msg='Real camera + YOLO + PX4 MAVSDK bridge active.'),
        LogInfo(msg='*** autonomy_manager_node owns /autonomy_enable and /mavsdk_offboard_enable ***'),
        LogInfo(msg='*** request autonomy with /autonomy_request; code still stays idle until target + safety gates pass ***'),
        LogInfo(msg="Request autonomy: ros2 topic pub --once /autonomy_request std_msgs/msg/Bool '{data: true}'"),
        LogInfo(msg="Disable autonomy: ros2 topic pub --once /autonomy_request std_msgs/msg/Bool '{data: false}'"),
        camera,
        yolo,
        tracker,
        telemetry,
        autonomy_manager,
        control,
        visualizer,
        health_monitor,
    ])