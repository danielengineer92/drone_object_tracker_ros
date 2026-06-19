"""
Launch file for simulation mode.

Launches fake camera, fake detections, fake telemetry, tracker, control,
and visualizer. No hardware required. Useful for development and testing.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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


    dashboard_arg = DeclareLaunchArgument(
        'dashboard',
        default_value='true',
        description='Launch the lightweight web dashboard node.'
    )

    dashboard_port_arg = DeclareLaunchArgument(
        'dashboard_port',
        default_value='8080',
        description='HTTP port for the dashboard.'
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


    dashboard = Node(
        package='drone_dashboard',
        executable='dashboard_node',
        name='dashboard_node',
        parameters=[config_file, {'port': ParameterValue(LaunchConfiguration('dashboard_port'), value_type=int)}],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('dashboard')),
    )

    return LaunchDescription([
        headless_arg,
        dashboard_arg,
        dashboard_port_arg,
        target_class_arg,
        LogInfo(msg='=== DRONE VISION SYSTEM - SIMULATION MODE ==='),
        LogInfo(msg='No hardware required. All data is simulated.'),
        LogInfo(msg=['Dashboard: http://<pi-ip>:', LaunchConfiguration('dashboard_port'), '/']),
        LogInfo(msg='autonomy request is FALSE by default. Request autonomy with /drone/autonomy/request or the dashboard button.'),
        fake_camera,
        fake_detection,
        fake_telemetry,
        tracker,
        autonomy_manager,
        control,
        visualizer,
        health_monitor,
        dashboard,
    ])