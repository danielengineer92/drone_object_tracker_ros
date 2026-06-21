"""
Launch file for PX4 SITL orbit testing.

Brings up the orbit mission against a real PX4 SITL connection, but with the
vision stack faked so no camera/YOLO is needed:

    telemetry_node     real MAVSDK bridge to PX4 SITL (udp://:14540)
    fake_detection     stationary, centered target (so the tracker locks)
    tracker_node       detections -> target_error + distance estimate
    autonomy_manager   safety gates
    control_node       position-hold + yaw
    mission_executor   walks the orbit_red_ball.yaml plan
    dashboard          System Ready / Start Mission buttons
    health_monitor     topic heartbeat

Prereqs (operator):
    1. PX4 SITL running, with mavproxy routing an output to udp 127.0.0.1:14540
       (e.g. mavproxy.py ... --out=udpout:127.0.0.1:14540).
    2. The vehicle ARMED (QGC or mavproxy `arm throttle`). telemetry_node does
       not arm; preflight blocks until it sees armed.

Then:
    ros2 launch drone_bringup sitl_orbit_launch.py
    # in the dashboard (or via topics): System Ready -> Start Mission

WARNING: allow_mavsdk_actions defaults to TRUE here because that is the whole
point of SITL testing (TAKEOFF / DO_ORBIT / LAND must reach PX4). Do not reuse
this launch file on real hardware without understanding that.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Generate the SITL orbit launch description."""

    bringup_dir = get_package_share_directory('drone_bringup')
    control_dir = get_package_share_directory('drone_control')
    config_file = os.path.join(bringup_dir, 'config', 'full_system_params.yaml')
    default_plan = os.path.join(control_dir, 'missions', 'orbit_red_ball.yaml')

    # Launch arguments
    connection_url_arg = DeclareLaunchArgument(
        'connection_url',
        default_value='udp://:14540',
        description='MAVSDK connection URL to PX4 SITL (mavproxy should --out to this port).'
    )

    allow_mavsdk_actions_arg = DeclareLaunchArgument(
        'allow_mavsdk_actions',
        default_value='true',
        description='SITL: allow TAKEOFF/LAND/RTL/DO_ORBIT actions to reach PX4. True by default here.'
    )

    mission_plan_file_arg = DeclareLaunchArgument(
        'mission_plan_file',
        default_value=default_plan,
        description='YAML mission plan. Defaults to the installed orbit_red_ball.yaml.'
    )

    require_distance_for_orbit_arg = DeclareLaunchArgument(
        'require_distance_for_orbit',
        default_value='false',
        description='If false, orbit proceeds even without a valid distance estimate '
                    '(PX4 then orbits the current position). Handy for a first SITL run.'
    )

    target_class_arg = DeclareLaunchArgument(
        'target_class',
        default_value='red_ball',
        description='Fake target class (must match tracker target_class).'
    )

    motion_pattern_arg = DeclareLaunchArgument(
        'motion_pattern',
        default_value='stationary',
        description='Fake target motion. stationary keeps it centered so track_center clears quickly.'
    )

    dashboard_arg = DeclareLaunchArgument(
        'dashboard',
        default_value='true',
        description='Launch the web dashboard node.'
    )

    dashboard_port_arg = DeclareLaunchArgument(
        'dashboard_port',
        default_value='8080',
        description='HTTP port for the dashboard.'
    )

    record_bag_arg = DeclareLaunchArgument(
        'record_bag',
        default_value='false',
        description='Record a rosbag of mission/telemetry/target/control/MAVSDK topics.'
    )

    bag_output_arg = DeclareLaunchArgument(
        'bag_output',
        default_value='bags/sitl_orbit',
        description='ros2 bag output path. Use a fresh path each run.'
    )

    # Nodes
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
        emulate_tty=True,
    )

    fake_detection = Node(
        package='drone_fake',
        executable='fake_detection_node',
        name='fake_detection_node',
        parameters=[{
            'detections_topic': '/drone/vision/detections',
            'target_class': LaunchConfiguration('target_class'),
            'motion_pattern': LaunchConfiguration('motion_pattern'),
            'add_false_detections': False,
            'detection_dropout_rate': 0.0,
            'publish_rate': 15.0,
        }],
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

    mission_executor = Node(
        package='drone_control',
        executable='mission_executor_node',
        name='mission_executor_node',
        parameters=[
            config_file,
            {
                'mission_plan_file': LaunchConfiguration('mission_plan_file'),
                'require_distance_for_orbit': ParameterValue(
                    LaunchConfiguration('require_distance_for_orbit'), value_type=bool
                ),
            },
        ],
        output='screen',
        emulate_tty=True,
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

    bag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record', '-o', LaunchConfiguration('bag_output'),
            '/drone/mission/state',
            '/drone/mission/command',
            '/drone/mission/request',
            '/drone/autonomy/state',
            '/drone/autonomy/request',
            '/drone/autonomy/enabled',
            '/drone/mavsdk/offboard_request',
            '/drone/mavsdk/offboard_enable',
            '/drone/mavsdk/action_command',
            '/drone/mavsdk/command_status',
            '/drone/telemetry',
            '/drone/tracking/target_error',
            '/drone/vision/detections',
            '/drone/control/command',
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('record_bag')),
    )

    return LaunchDescription([
        connection_url_arg,
        allow_mavsdk_actions_arg,
        mission_plan_file_arg,
        require_distance_for_orbit_arg,
        target_class_arg,
        motion_pattern_arg,
        dashboard_arg,
        dashboard_port_arg,
        record_bag_arg,
        bag_output_arg,
        LogInfo(msg='=== DRONE - PX4 SITL ORBIT MODE ==='),
        LogInfo(msg='Real MAVSDK bridge + FAKE vision (no camera/YOLO).'),
        LogInfo(msg=['Connecting to PX4 at ', LaunchConfiguration('connection_url'),
                     ' | mission plan: ', LaunchConfiguration('mission_plan_file')]),
        LogInfo(msg='*** allow_mavsdk_actions defaults TRUE for SITL. Arm via QGC/mavproxy first. ***'),
        LogInfo(msg=['Dashboard: http://<wsl-ip>:', LaunchConfiguration('dashboard_port'),
                     '/  then: System Ready -> Start Mission']),
        LogInfo(msg='Watch: ros2 topic echo /drone/mission/state  and  /drone/mavsdk/command_status'),
        telemetry,
        fake_detection,
        tracker,
        autonomy_manager,
        control,
        mission_executor,
        health_monitor,
        dashboard,
        bag_record,
    ])
