# Position Hold + Yaw Stage

This patch changes the smart mission control path from body-frame velocity yaw commands to local-NED position-hold yaw commands.

## What changed

- `ControlCommand.msg` now supports `POSITION` commands:
  - `position_valid`
  - `position_north`
  - `position_east`
  - `position_down`
  - `yaw_deg`
- `DroneTelemetry.msg` now publishes PX4 local-NED position:
  - `local_position_valid`
  - `local_position_north`
  - `local_position_east`
  - `local_position_down`
- `control_node.py` captures the local-NED position when autonomy starts and keeps reusing that anchor.
- The vision target error now changes an absolute yaw target while the held N/E/D position stays fixed.
- `telemetry_node.py` streams `position_velocity_ned()` from MAVSDK and sends `PositionNedYaw` Offboard setpoints.
- Legacy `VELOCITY` commands are still supported, but the smart mission now emits `POSITION` commands.
- Fake telemetry and dashboard JSON were updated so sim/dev mode can display and publish the new fields.

## Behavior

Stage 1 behavior is now:

1. Wait for PX4 telemetry and valid local-NED position.
2. When autonomy is enabled, capture the current local-NED position as the hold anchor.
3. Publish `POSITION` commands using that fixed N/E/D anchor.
4. When the target is visible, integrate the yaw-rate controller into an absolute `yaw_deg` setpoint.
5. When the target is missing/stale, keep holding the same position and current yaw target.

This intentionally disables forward/right/down translation for Stage 1 so the vehicle holds its takeoff/local position and only yaws.
