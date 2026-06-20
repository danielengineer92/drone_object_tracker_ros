"""
Launch file for vision mode.

Launches real camera, YOLO detection, tracker, control, fake telemetry, and visualizer.
Requires a camera but no drone connection.
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
        default_value='/home/robotpi/drone_ws/src/drone_object_tracker_ros/models/red_ball_ncnn_model',
        description='Path to YOLO model file'
    )

    target_class_arg = DeclareLaunchArgument(
        'target_class',
        default_value='red_ball',
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
        camera_index_arg,
        model_path_arg,
        target_class_arg,
        LogInfo(msg='=== DRONE VISION SYSTEM - VISION MODE ==='),
        LogInfo(msg='Real camera + YOLO detection active.'),
        LogInfo(msg=['Dashboard: http://<pi-ip>:', LaunchConfiguration('dashboard_port'), '/']),
        LogInfo(msg='autonomy request is FALSE - /drone/control/command stays IDLE until the manager publishes /drone/autonomy/enabled true.'),
        camera,
        yolo,
        tracker,
        autonomy_manager,
        control,
        visualizer,
        health_monitor,
        dashboard,
    ])
