"""
Launch file for full system mode.

Launches real camera, YOLO detection, tracker, control, PX4 MAVSDK bridge,
and visualizer. Requires camera and PX4 drone connection.

WARNING: autonomy_enabled defaults to False and mavsdk_offboard_enabled defaults to False.
Commands stay IDLE until /drone/autonomy/enabled is true, and nothing is sent to PX4
until /drone/mavsdk/offboard_request is requested and /drone/mavsdk/offboard_enable becomes true.
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
        default_value='/home/robotpi/drone_ws/src/drone_object_tracker_ros/models/red_ball_ncnn_model',
        description='Path to YOLO model file'
    )

    target_class_arg = DeclareLaunchArgument(
        'target_class',
        default_value='red_ball',
        description='Target class to track'
    )

    connection_url_arg = DeclareLaunchArgument(
        'connection_url',
        default_value='serial:///dev/ttyACM0:57600',
        description='MAVSDK connection URL to PX4'
    )

    allow_mavsdk_actions_arg = DeclareLaunchArgument(
        'allow_mavsdk_actions',
        default_value='false',
        description='SITL/dev only: allow mission TAKEOFF/LAND/RTL/HOLD actions through MAVSDK'
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

    telemetry = Node(
        package='drone_telemetry',
        executable='telemetry_node',
        name='telemetry_node',
        parameters=[
            config_file,
            {
                'connection_url': LaunchConfiguration('connection_url'),
                'allow_mavsdk_actions': ParameterValue(LaunchConfiguration('allow_mavsdk_actions'), value_type=bool),
            },
        ],
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

    mission_executor = Node(
        package='drone_control',
        executable='mission_executor_node',
        name='mission_executor_node',
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
        connection_url_arg,
        allow_mavsdk_actions_arg,
        LogInfo(msg='=== DRONE VISION SYSTEM - FULL SYSTEM MODE ==='),
        LogInfo(msg='Real camera + YOLO + PX4 MAVSDK bridge active.'),
        LogInfo(msg=['Dashboard: http://<pi-ip>:', LaunchConfiguration('dashboard_port'), '/']),
        LogInfo(msg='*** autonomy_manager_node owns /drone/autonomy/enabled and /drone/mavsdk/offboard_enable; request MAVSDK on /drone/mavsdk/offboard_request ***'),
        LogInfo(msg='*** mission_executor_node owns Start Mission: preflight -> takeoff if needed -> prime Offboard -> TRACK_CENTER yaw ***'),
        LogInfo(msg='*** dashboard main buttons: System Ready, Start Mission, Abort/Hold, Land; debug gates are in the drawer ***'),
        LogInfo(msg='*** SITL takeoff/land via Start Mission needs: allow_mavsdk_actions:=true. Leave false for normal hardware bench tests. ***'),
        LogInfo(msg="Request autonomy: ros2 topic pub --once /drone/autonomy/request std_msgs/msg/Bool '{data: true}'"),
        LogInfo(msg="Disable autonomy: ros2 topic pub --once /drone/autonomy/request std_msgs/msg/Bool '{data: false}'"),
        camera,
        yolo,
        tracker,
        telemetry,
        autonomy_manager,
        mission_executor,
        control,
        visualizer,
        health_monitor,
        dashboard,
    ])