# Drone Dashboard

This patch adds a lightweight web dashboard node that runs inside ROS 2 with no
Node.js, Flask, or npm dependency.

## Launch

The dashboard is enabled by default in the bringup launch files:

```bash
ros2 launch drone_bringup simulation_launch.py
ros2 launch drone_bringup vision_launch.py
ros2 launch drone_bringup full_system_launch.py
```

Open it from a laptop on the same network:

```text
http://<raspberry-pi-ip>:8080/
```

Change the port:

```bash
ros2 launch drone_bringup full_system_launch.py dashboard_port:=8081
```

Disable it:

```bash
ros2 launch drone_bringup full_system_launch.py dashboard:=false
```

## What it shows

- mission/autonomy state from `/drone/mission/state`
- autonomy gate from `/drone/autonomy/enabled`
- MAVSDK Offboard gate from `/drone/mavsdk/offboard_enable`
- PX4 telemetry from `/drone/telemetry`
- target status from `/drone/tracking/target_error`
- control output from `/drone/control/command`
- detection count from `/drone/vision/detections`

## Buttons

The dashboard has request/disable buttons that publish to:

```text
/drone/autonomy/request
/drone/mavsdk/offboard_request
/drone/mission/request
```

The dashboard does **not** arm, take off, land, or bypass the safety gates. It
asks the autonomy manager for autonomy and separately asks for MAVSDK Offboard.
The autonomy manager still decides whether `/drone/autonomy/enabled` and
`/drone/mavsdk/offboard_enable` become true.


## Mission request API

The dashboard now exposes a mission request endpoint that publishes `std_msgs/Bool` to `/drone/mission/request`:

```bash
curl -X POST http://<pi-ip>:8080/api/mission_request \
  -H 'Content-Type: application/json' \
  -d '{"enabled": true}'
```

Stop the mission request with:

```bash
curl -X POST http://<pi-ip>:8080/api/mission_request \
  -H 'Content-Type: application/json' \
  -d '{"enabled": false}'
```

The MAVSDK request endpoint publishes `std_msgs/Bool` to `/drone/mavsdk/offboard_request`:

```bash
curl -X POST http://<pi-ip>:8080/api/mavsdk_request \
  -H 'Content-Type: application/json' \
  -d '{"enabled": true}'
```

The older autonomy gate endpoint remains `/api/autonomy_request`, which publishes to `/drone/autonomy/request`.
