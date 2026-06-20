# Mission + Distance Upgrade

This update adds the first real scaffolding for the ball-orbit-return mission while keeping the real flight/action gates off by default.

## Added

### Distance estimation

`tracker_node.py` now publishes these extra fields on `/drone/tracking/target_error`:

- `distance_valid`
- `distance_m`
- `raw_distance_m`
- `bearing_x_rad`
- `bearing_y_rad`
- `target_diameter_px`

The distance model is:

```text
distance_m = distance_calibration_k / target_diameter_px
```

If `distance_calibration_k` is left at `0.0`, the tracker falls back to:

```text
distance_m = ball_diameter_m * focal_length_px / target_diameter_px
```

or FOV-derived focal length if `focal_length_px` is also `0.0`.

### Mission command layer

New message:

```text
/drone/mission/command  drone_interfaces/msg/MissionCommand
```

`control_node.py` now understands these mission modes:

- `IDLE`
- `HOLD`
- `TRACK_CENTER`
- `FLY_FORWARD`
- `APPROACH_TARGET`
- `ORBIT_TARGET`

Translation outputs are still blocked in `telemetry_node.py` unless `allow_translation_commands` is explicitly enabled.

### MAVSDK action request layer

New message:

```text
/drone/mavsdk/action_command  drone_interfaces/msg/MavsdkActionCommand
```

`telemetry_node.py` can receive one-shot requests for:

- `ARM`
- `TAKEOFF`
- `LAND`
- `RETURN_TO_LAUNCH`
- `HOLD`
- `DO_ORBIT`

These are blocked by default with:

```yaml
allow_mavsdk_actions: false
```

### Mission executor

New node:

```text
mission_executor_node = drone_control.mission_executor_node:main
```

It sequences:

```text
1. TAKEOFF
2. FLY_FORWARD
3. WAIT_FOR_TARGET
4. APPROACH_TARGET
5. DO_ORBIT or visual ORBIT_TARGET fallback
6. RETURN_TO_LAUNCH
7. LAND
```

The node is launched by `full_system_launch.py` but disabled by default:

```yaml
mission_executor_node:
  ros__parameters:
    mission_enabled: false
```

## Safety defaults

The update keeps these physical-output gates off by default:

```yaml
control_node.autonomy_enabled: false
telemetry_node.mavsdk_offboard_enabled: false
telemetry_node.allow_translation_commands: false
telemetry_node.allow_mavsdk_actions: false
mission_executor_node.mission_enabled: false
```

So this update is safe to build and inspect without suddenly enabling takeoff/orbit/land actions.

## MavlinkDirect orbit update

`DO_ORBIT` is now sent as raw `COMMAND_LONG` / `MAV_CMD_DO_ORBIT` through MAVSDK `MavlinkDirect` instead of `action.do_orbit()`.

Reason: MAVSDK's high-level `action.do_orbit()` does not expose the MAVLink orbit amount field, but `MAV_CMD_DO_ORBIT` param4 does.

Mapping:

```text
param1 = radius_m
param2 = velocity_m_s
param3 = yaw_behavior enum value
param4 = orbit_revolutions * 2*pi   # radians; 0 means orbit forever
param5 = latitude_deg or JSON null  # current/default center when null
param6 = longitude_deg or JSON null
param7 = absolute_altitude_m or JSON null
```

The telemetry node intentionally raises an action failure if the installed MAVSDK-Python package does not expose `drone.mavlink_direct.send_message()`. It does not silently fall back to an infinite orbit.

Quick import/API check on the Pi:

```bash
source ~/venv/bin/activate
python3 - <<'PY'
from mavsdk import System
import mavsdk.mavlink_direct as md

drone = System()
print('mavlink_direct module:', md)
print('message classes:', [x for x in dir(md) if 'Message' in x or 'Mavlink' in x])
print('System has mavlink_direct:', hasattr(drone, 'mavlink_direct'))
if hasattr(drone, 'mavlink_direct'):
    print('direct methods:', [x for x in dir(drone.mavlink_direct) if 'send' in x.lower() or 'message' in x.lower()])
PY
```


## Starting the mission

Dashboard API:

```bash
curl -X POST http://<pi-ip>:8080/api/mission_request \
  -H 'Content-Type: application/json' \
  -d '{"enabled": true}'
```

CLI-only path:

```bash
ros2 topic pub --once /drone/mission/request std_msgs/msg/Bool '{data: true}'
```

Stop it with the same topic/API using `false`.
