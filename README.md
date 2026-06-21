# Drone Object Tracker ROS

ROS 2 Jazzy workspace for a Raspberry Pi + PX4 object-tracking drone. The current mission is intentionally conservative: take off, enter PX4 Offboard with a local-position hold, then yaw in place to center a YOLO target. Forward/strafe/orbit behavior is present as later mission scaffolding, but disabled by default.

## What It Does

- Publishes camera frames from a USB/V4L2 camera.
- Runs YOLO detection, typically against the `red_ball` model.
- Tracks the target and publishes normalized image error.
- Bridges PX4 telemetry and Offboard commands through MAVSDK.
- Runs a safety-gated mission state machine.
- Provides a lightweight web dashboard at `http://<pi-ip>:8080/`.
- Holds local NED position while yawing toward the target.

For the full mental model of how the nodes, topics, gates, dashboard, logs, and bags fit together, see `ARCHITECTURE.md`.

## Current Mission Flow

The dashboard exposes the pilot flow:

```text
System Ready -> Start Mission -> Abort/Hold -> Land
```

`Start Mission` publishes `/drone/mission/request=true`. The mission executor then runs:

```text
PREFLIGHT -> TAKEOFF -> PRIME_OFFBOARD -> TRACK_CENTER
```

Current defaults:

- Takeoff target: `3.0 m`
- Offboard-ready threshold: `2.7 m`
- Tracking mode: yaw-only target centering
- Position anchor: captured during `PRIME_OFFBOARD` and reused when tracking starts
- Translation commands: disabled by default
- MAVSDK actions: blocked unless `allow_mavsdk_actions:=true`

## Safety Model

There are separate gates on purpose:

- `/drone/mission/request`: operator asks the mission executor to start or stop.
- `/drone/autonomy/request`: mission/dashboard asks the autonomy manager to allow autonomy.
- `/drone/autonomy/enabled`: autonomy manager allows the control node to generate approved commands.
- `/drone/mavsdk/offboard_request`: mission/dashboard asks for MAVSDK Offboard.
- `/drone/mavsdk/offboard_enable`: autonomy manager allows the MAVSDK bridge to start/send Offboard setpoints.

When autonomy is blocked, `control_node` publishes `IDLE` with zero motion.

The MAVSDK bridge does not arm the drone. Arm from RC/QGroundControl or your normal safe hardware procedure.

## Packages

```text
src/drone_camera       Camera publisher
src/drone_yolo         YOLO detector
src/drone_tracker      Target selection, debounce, distance estimate
src/drone_telemetry    PX4 MAVSDK telemetry, actions, Offboard bridge
src/drone_control      Autonomy manager, mission executor, control commands
src/drone_dashboard    Web dashboard and pilot controls
src/drone_visualizer   Optional OpenCV visualization
src/drone_diagnostics  Topic heartbeat/status helpers
src/drone_fake         Fake camera/detections/telemetry for simulation
src/drone_bringup      Launch files and YAML configs
src/drone_interfaces   Custom ROS messages
```

## Hardware

- Raspberry Pi 4 or better
- Ubuntu 24.04
- ROS 2 Jazzy
- PX4 flight controller, tested target is Pixhawk 6C-class hardware
- USB camera or Pi camera exposed through OpenCV/V4L2
- Serial link from Pi to PX4, default `serial:///dev/ttyACM0:57600`

## Install

Create a ROS workspace and clone/copy this repo into `src`:

```bash
mkdir -p ~/drone_ws/src
cd ~/drone_ws/src
git clone -b position-hold-yaw https://github.com/danielengineer92/drone_object_tracker_ros.git
cd ~/drone_ws
```

Install ROS/system dependencies:

```bash
sudo apt update
sudo apt install -y \
  python3-pip \
  python3-opencv \
  libopencv-dev \
  ros-jazzy-cv-bridge \
  ros-jazzy-sensor-msgs \
  ros-jazzy-std-msgs
```

Install Python dependencies:

```bash
pip install -r src/drone_object_tracker_ros/requirements.txt
```

Build:

```bash
cd ~/drone_ws
colcon build --symlink-install
source install/setup.bash
```

## Launch Modes

Simulation/fake nodes:

```bash
ros2 launch drone_bringup simulation_launch.py
```

Camera/vision only:

```bash
ros2 launch drone_bringup vision_launch.py camera_index:=0 model_path:=yolov8n.pt target_class:=person
```

Full system:

```bash
ros2 launch drone_bringup full_system_launch.py \
  connection_url:="serial:///dev/ttyACM0:57600" \
  camera_index:=0 \
  target_class:=red_ball
```

SITL/dev mode with MAVSDK takeoff/land actions allowed:

```bash
ros2 launch drone_bringup full_system_launch.py \
  connection_url:="udp://:14540" \
  allow_mavsdk_actions:=true
```

PX4 SITL orbit test (real MAVSDK bridge + faked vision, no camera/YOLO):

```bash
ros2 launch drone_bringup sitl_orbit_launch.py
```

Prereqs: PX4 SITL running with a MAVLink route to `udp 127.0.0.1:14540` (e.g.
`mavproxy.py ... --out=udpout:127.0.0.1:14540`), and the vehicle armed (QGC or
mavproxy `arm throttle`). It runs the `orbit_red_ball.yaml` plan with a fake
centered target and `allow_mavsdk_actions:=true`, so `Start Mission` walks
takeoff -> prime_offboard -> track_center -> orbit -> land. `require_distance_for_orbit`
defaults to false so the first run reaches orbit even without a distance estimate
(PX4 then orbits the current position). Watch `/drone/mission/state` and
`/drone/mavsdk/command_status`.

Full system with a rosbag for later review:

```bash
ros2 launch drone_bringup full_system_launch.py \
  connection_url:="serial:///dev/ttyACM0:57600" \
  record_bag:=true \
  bag_output:=bags/field_test_001
```

Use a new `bag_output` path for each run. The launch file records mission, telemetry, target, detection, control, autonomy, and MAVSDK status topics, but skips raw images by default to protect disk space.

For real hardware bench tests, leave `allow_mavsdk_actions:=false` unless you intentionally want dashboard/mission TAKEOFF, LAND, RTL, and HOLD actions to reach PX4.

## Dashboard

Open:

```text
http://<raspberry-pi-ip>:8080/
```

Buttons:

- `System Ready`: requests autonomy readiness.
- `Start Mission`: starts preflight, takeoff, Offboard prime, then yaw tracking.
- `Abort / Hold`: drops mission/autonomy/offboard requests and sends HOLD if MAVSDK actions are allowed.
- `Land`: drops mission/autonomy/offboard requests and sends LAND if MAVSDK actions are allowed.

The dashboard is intentionally lightweight now. The raw JSON console is collapsed by default to reduce browser load.

The `Preflight` panel gives a fast go/no-go view before `Start Mission`:

- `Telemetry Fresh`, `PX4 Link`, and `Armed` must be healthy before mission preflight can pass.
- `Battery` expects at least 25%.
- `Local Position` should be valid before position-hold/yaw tracking.
- `Vision Fresh` and `Target Lock` tell you whether the detector/tracker is feeding current data.

Important: the dashboard binds to `0.0.0.0` by default. Use it only on a trusted network until authentication/token protection is added.

## Key Parameters

Full-system config lives in:

```text
src/drone_bringup/config/full_system_params.yaml
```

Common settings:

```yaml
yolo_node:
  ros__parameters:
    model_path: "/home/robotpi/drone_ws/src/drone_object_tracker_ros/models/red_ball_ncnn_model"
    target_class: "red_ball"
    input_size: 320

telemetry_node:
  ros__parameters:
    connection_url: "serial:///dev/ttyACM0:57600"
    allow_mavsdk_actions: false
    allow_translation_commands: false

mission_executor_node:
  ros__parameters:
    mission_enabled: true
    takeoff_altitude_m: 3.0
    airborne_altitude_m: 2.7
    event_log_enabled: true
    event_log_directory: "~/drone_mission_logs"
    run_full_orbit_after_track_center: false

control_node:
  ros__parameters:
    gain_yaw: 0.8
    max_yaw_rate: 1.0
    require_armed: true
    require_gps: false
```

## Position Hold / Yaw Behavior

The mission primes Offboard before target tracking:

1. `TAKEOFF` requests PX4 takeoff to `3.0 m`.
2. Mission waits until relative altitude is at least `2.7 m`.
3. `PRIME_OFFBOARD` requests MAVSDK Offboard and publishes `HOLD`.
4. `telemetry_node` captures a local NED hold setpoint and keeps PX4 holding there.
5. `control_node` also captures the same local-position stream during `PRIME_OFFBOARD` as `mission_prime_offboard`.
6. When target lock turns `/drone/autonomy/enabled` true, `control_node` reuses that captured anchor and only updates yaw.

In logs, look for:

```text
Captured bridge prime POSITION hold
Captured POSITION hold anchor (mission_prime_offboard)
anchor_valid=True, anchor_source=mission_prime_offboard
```

## Programming Missions

The mission sequence is data-driven. The executor walks an ordered list of step
"verbs" instead of a hardcoded state chain, so you can define or reorder a flight
script — including orbiting a target — by editing a YAML file, no code changes.

With no plan file, the built-in default plan runs and behavior is unchanged.

Provide a plan at launch:

```bash
ros2 launch drone_bringup full_system_launch.py \
  connection_url:="udp://:14540" \
  allow_mavsdk_actions:=true \
  mission_plan_file:=src/drone_control/missions/orbit_red_ball.yaml
```

Example plan (`src/drone_control/missions/orbit_red_ball.yaml`):

```yaml
mission:
  name: orbit_red_ball
  steps:
    - {type: takeoff, altitude_m: 3.0}
    - {type: prime_offboard, hold_s: 1.5}
    - {type: track_center, until: centered, timeout_s: 20}
    - {type: orbit, radius_m: 2.0, speed_m_s: 0.4, revolutions: 1}
    - {type: land}
```

Step verbs (each maps to an existing, safety-gated primitive):

| Verb | Does | Key params |
|---|---|---|
| `takeoff` | MAVSDK takeoff if not already airborne | `altitude_m` |
| `prime_offboard` | Request Offboard, publish hold setpoints | `hold_s` |
| `track_center` | Hold position, yaw toward the YOLO target | `until`, `timeout_s` |
| `approach` | Later-stage distance approach (scaffold) | `timeout_s` |
| `orbit` | PX4 `MAV_CMD_DO_ORBIT` around the target | `radius_m`, `speed_m_s`, `revolutions`, `timeout_s` |
| `rtl` | Return to launch | `timeout_s` |
| `land` | MAVSDK land | `timeout_s` |
| `hold` | Hold position for a fixed time | `status`, `timeout_s` |

A step advances when its `until` condition is met **or** its `timeout_s` elapses.
Valid `until` values: `airborne`, `centered`, `locked`, `approach_done`, `none`
(`none` = never auto-advance; hold until stopped or timeout). Any omitted param
falls back to the node's `mission_executor_node` parameters.

The plan is validated on load. An invalid plan logs an error and falls back to the
built-in default rather than crashing, and a motion step placed before
`prime_offboard` logs a lint warning.

Important: `orbit` does not bypass any safety gate. It still needs
`allow_mavsdk_actions:=true`, GPS / valid global position, and a valid distance
estimate (`distance_valid` from the tracker). Without those, PX4 cannot place the
orbit center.

See `src/drone_control/missions/` for examples and
`src/drone_control/drone_control/mission_plan.py` for the schema.

## Useful Commands

Mission event logs:

```bash
ls -lh ~/drone_mission_logs/
tail -f ~/drone_mission_logs/mission_events_*.jsonl
```

Record/play a bag manually:

```bash
ros2 bag record -o bags/manual_test /drone/mission/state /drone/telemetry /drone/tracking/target_error /drone/control/command /drone/mavsdk/command_status
ros2 bag play bags/manual_test
```

Watch mission/autonomy state:

```bash
ros2 topic echo /drone/mission/state
ros2 topic echo /drone/autonomy/state
```

Watch PX4 bridge status:

```bash
ros2 topic echo /drone/mavsdk/command_status
```

Watch target and command output:

```bash
ros2 topic echo /drone/tracking/target_error
ros2 topic echo /drone/control/command
```

Manually request/stop the mission:

```bash
ros2 topic pub --once /drone/mission/request std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /drone/mission/request std_msgs/msg/Bool "{data: false}"
```

Manually request/stop autonomy:

```bash
ros2 topic pub --once /drone/autonomy/request std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /drone/autonomy/request std_msgs/msg/Bool "{data: false}"
```

List graph:

```bash
ros2 node list
ros2 topic list
ros2 node info /control_node
```

## Troubleshooting

Takeoff does not happen:

- Check `allow_mavsdk_actions`. It must be true for mission TAKEOFF to reach PX4.
- Check `/drone/mavsdk/command_status`.
- Check PX4 is connected and armed.
- Check the mission is not stuck in `PREFLIGHT`.

Mission starts Offboard too early:

- Verify `takeoff_altitude_m: 3.0` and `airborne_altitude_m: 2.7`.
- Watch `/drone/mission/state`; it should report altitude while waiting in `TAKEOFF`.

Target never locks:

- Check YOLO model path and target class.
- Echo `/drone/vision/detections`.
- Echo `/drone/tracking/target_error`.
- Lower `confidence_threshold` only in a controlled test.

Yaw command never reaches PX4:

- Check `/drone/autonomy/state` reaches `TRACKING`.
- Check `/drone/mavsdk/offboard_enable` is true.
- Check `/drone/control/command` has `command_type: POSITION`, `position_valid: true`, and `executed: true`.
- Check `/drone/mavsdk/command_status`.

Dashboard is slow:

- Keep the raw snapshot console closed.
- Use the current lightweight dashboard code.
- Avoid running the OpenCV visualizer UI on the Pi unless needed.

## Development Checks

Basic syntax check:

```bash
python3 -m compileall -q src main.py
```

Build after message/interface changes:

```bash
colcon build --symlink-install
source install/setup.bash
```

If you change `.msg` files, rebuild and restart any running ROS nodes so generated interfaces match.
