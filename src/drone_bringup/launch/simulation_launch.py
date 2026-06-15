"""
Launch file for simulation mode.

Launches fake camera, fake detections, fake telemetry, tracker, control,
and visualizer. No hardware required. Useful for development and testing.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate the simulation launch description."""

    # Get config file path
    bringup_dir = get_package_share_directory('drone_bringup')
    config_file = os.path.join(bringup_dir, 'config', 'simulation_params.yaml')

    # Launch arguments
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='true',
        description='Run without display window. Use headless:=false only when a desktop display is available.'
    )

    target_class_arg = DeclareLaunchArgument(
        'target_class',
        default_value='person',
        description='Target class to track'
    )

    # Nodes
    fake_camera = Node(
        package='drone_fake',
        executable='fake_camera_node',
        name='fake_camera_node',
        parameters=[config_file],
        output='screen',
    )

    fake_detection = Node(
        package='drone_fake',
        executable='fake_detection_node',
        name='fake_detection_node',
        parameters=[config_file, {'target_class': LaunchConfiguration('target_class')}],
        output='screen',
    )

    fake_telemetry = Node(
        package='drone_fake',
        executable='fake_telemetry_node',
        name='fake_telemetry_node',
        parameters=[config_file],
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
        target_class_arg,
        LogInfo(msg='=== DRONE VISION SYSTEM - SIMULATION MODE ==='),
        LogInfo(msg='No hardware required. All data is simulated.'),
        LogInfo(msg='autonomy_enabled is FALSE by default. Enable yaw-only sim commands with /autonomy_enable.'),
        fake_camera,
        fake_detection,
        fake_telemetry,
        tracker,
        control,
        visualizer,
        health_monitor,
    ])