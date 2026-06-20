# Autonomy Gating Safety Update

This update adds a mission/autonomy state machine and topic-based autonomy gates to the ROS 2 drone vision pipeline.

## Pipeline

```text
Camera -> YOLO -> Tracker -> /drone/tracking/target_error -> Control Node -> /drone/control/command -> MAVSDK bridge -> Pixhawk
                         /drone/autonomy/enabled ----^ 
                         /drone/telemetry -----^
```

## Topic names

| Topic | Type | Direction | Purpose |
|---|---|---:|---|
| `/drone/camera/image_raw` | `sensor_msgs/Image` | Camera -> YOLO | Camera frames |
| `/drone/vision/detections` | `drone_interfaces/DetectionArray` | YOLO -> Tracker | Object detections |
| `/drone/tracking/target_error` | `drone_interfaces/TargetError` | Tracker -> Control | Normalized target error and lock state |
| `/drone/autonomy/request` | `std_msgs/Bool` | Operator/dashboard -> Autonomy manager | Requests autonomy |
| `/drone/mission/state` | `std_msgs/String` | Autonomy manager -> Operator/dashboard | Mission state and block reason |
| `/drone/autonomy/state` | `std_msgs/String` | Autonomy manager -> Operator/dashboard | Backward-compatible autonomy state |
| `/drone/autonomy/enabled` | `std_msgs/Bool` | Autonomy manager -> Control | Enables/disables movement generation |
| `/drone/telemetry` | `drone_interfaces/DroneTelemetry` | MAVSDK telemetry -> Control/manager | Drone safety state |
| `/drone/control/command` | `drone_interfaces/ControlCommand` | Control -> MAVSDK bridge | Zero/idle or yaw-only command output |
| `/drone/mavsdk/offboard_request` | `std_msgs/Bool` | Operator/dashboard -> Autonomy manager | Requests the MAVSDK Offboard executor gate |
| `/drone/mavsdk/offboard_enable` | `std_msgs/Bool` | Autonomy manager -> MAVSDK bridge | Enables/disables actual PX4 Offboard command sending after safety checks |
| `/drone/mavsdk/command_status` | `std_msgs/String` | MAVSDK bridge -> Operator/diagnostics | Shows whether commands are blocked, zeroed, or sent to PX4 |

## Control behavior

The control node always subscribes to and processes `/drone/tracking/target_error`, even when autonomy is disabled.

When `autonomy_enabled == false`:

- The control node publishes `ControlCommand.command_type = "IDLE"`.
- `velocity_forward`, `velocity_right`, `velocity_down`, and `yaw_rate` are all `0.0`.
- `executed = false`.
- `execution_status = "BLOCKED_AUTONOMY_DISABLED"`.

When `autonomy_enabled == true` and the target is locked:

- The control node converts horizontal target error into yaw only.
- Forward/back, lateral, altitude, and orbit commands remain `0.0`.
- The command is still blocked if telemetry safety gates fail.

When the target is lost, invisible, not `LOCKED`, or stale:

- The control node immediately publishes an idle/zero command.

## Enable / disable commands

Request yaw-only autonomy:

```bash
ros2 topic pub --once /drone/autonomy/request std_msgs/msg/Bool "{data: true}"
```

Disable autonomy:

```bash
ros2 topic pub --once /drone/autonomy/request std_msgs/msg/Bool "{data: false}"
```

Emergency software stop:

```bash
ros2 topic pub --once /drone/autonomy/request std_msgs/msg/Bool "{data: false}"
```


## Actual PX4 command sending

The control node does not talk to PX4 directly. It publishes `/drone/control/command`.

The existing `drone_telemetry` node is now a PX4 MAVSDK bridge. It owns the single Pixhawk connection and handles both telemetry and command sending. This avoids trying to open the same Pixhawk serial port from two separate MAVSDK nodes.

Actual PX4 Offboard command sending still has a second hard gate, but the autonomy manager owns it now. Watch it instead of manually forcing it during normal tests:

```bash
ros2 topic echo /drone/mavsdk/offboard_request
ros2 topic echo /drone/mavsdk/offboard_enable
```

Watch bridge status:

```bash
ros2 topic echo /drone/mavsdk/command_status
```

The bridge never arms or takes off. It only sends yaw-only `VelocityBodyYawspeed` Offboard setpoints after `/drone/control/command` is fresh, approved, and both gates are enabled.

## Bench test procedure

Build and source the workspace:

```bash
cd ~/drone_ws
colcon build --symlink-install
source /opt/ros/jazzy/setup.bash
source ~/drone_ws/install/setup.bash
```

Start the simulation pipeline:

```bash
ros2 launch drone_bringup simulation_launch.py
```

In another terminal, watch target error:

```bash
source /opt/ros/jazzy/setup.bash
source ~/drone_ws/install/setup.bash
ros2 topic echo /drone/tracking/target_error
```

In another terminal, watch control commands:

```bash
source /opt/ros/jazzy/setup.bash
source ~/drone_ws/install/setup.bash
ros2 topic echo /drone/control/command
```

Expected result before enabling autonomy:

```text
command_type: IDLE
yaw_rate: 0.0
executed: false
execution_status: BLOCKED_AUTONOMY_DISABLED
```

Enable autonomy:

```bash
ros2 topic pub --once /drone/autonomy/enabled std_msgs/msg/Bool "{data: true}"
```

Expected result with a locked target and passing telemetry safety gates:

```text
command_type: VELOCITY
velocity_forward: 0.0
velocity_right: 0.0
velocity_down: 0.0
yaw_rate: changes as /drone/tracking/target_error.error_x changes
executed: true
execution_status: SENT
```

Disable autonomy again:

```bash
ros2 topic pub --once /drone/autonomy/enabled std_msgs/msg/Bool "{data: false}"
```

Expected result after disabling:

```text
command_type: IDLE
yaw_rate: 0.0
executed: false
execution_status: BLOCKED_AUTONOMY_DISABLED
```

## Direct control-node test without camera/YOLO

Run fake telemetry in one terminal:

```bash
ros2 run drone_fake fake_telemetry_node --ros-args \
  -p simulate_armed:=true \
  -p simulate_flying:=true
```

Run the control node in another terminal:

```bash
ros2 run drone_control control_node --ros-args \
  -p require_armed:=false \
  -p min_battery_percent:=0.0
```

Publish a locked target error:

```bash
ros2 topic pub --rate 10 /drone/tracking/target_error drone_interfaces/msg/TargetError \
"{error_x: 0.5, error_y: 0.0, target_visible: true, target_class: 'person', target_confidence: 0.9, target_area: 0.1, tracking_state: 'LOCKED', time_since_last_seen: 0.0}"
```

Keep `/drone/control/command` echoing in another terminal. It should stay idle until you enable autonomy:

```bash
ros2 topic pub --once /drone/autonomy/enabled std_msgs/msg/Bool "{data: true}"
```

Now change the published `error_x` value. The `/drone/control/command.yaw_rate` value should change with it while all linear velocity fields stay zero.

## Logs to verify

Look for these control node logs:

```text
AUTONOMY DISABLED - publishing IDLE/zero commands until /drone/autonomy/enabled is true.
*** AUTONOMY ENABLED by /drone/autonomy/enabled ***
Target acquired by control node ...
Yaw command | error_x=... yaw_rate=... rad/s
Target lost by control node ...
Autonomy disabled by /drone/autonomy/enabled; publishing IDLE/zero commands.
```
