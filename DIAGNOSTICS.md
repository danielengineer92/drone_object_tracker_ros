# ROS 2 Diagnostics Guide

This workspace now includes lightweight diagnostics in every runtime node plus a dedicated health monitor.

## What logs now show

Every node prints startup configuration and a diagnostics heartbeat every 2-5 seconds. Heartbeats include:

- Topic message counts
- Topic rates in Hz
- Time since the last received message
- Publisher/subscriber connection counts
- Warnings for missing graph connections
- Warnings for stale inputs older than 2 seconds, or node-specific stale limits

## Health monitor

The `drone_diagnostics` package adds:

```bash
ros2 run drone_diagnostics health_monitor_node
```

It subscribes to the major pipeline topics:

- `/camera/image_raw`
- `/detections`
- `/target_error`
- `/drone/telemetry`
- `/control_command`

It is included automatically in:

```bash
ros2 launch drone_bringup vision_launch.py
ros2 launch drone_bringup simulation_launch.py
ros2 launch drone_bringup full_system_launch.py
```

## Typical debug commands

```bash
source /opt/ros/jazzy/setup.bash
source ~/drone_ws/install/setup.bash

ros2 launch drone_bringup vision_launch.py
ros2 topic list
ros2 node list
ros2 topic hz /camera/image_raw
ros2 topic hz /detections
ros2 topic hz /target_error
ros2 topic hz /control_command
ros2 topic echo /control_command
```

## How to read the logs

- `SYSTEM HEALTH OK` means the major topics have publishers and fresh data.
- `NO_PUBLISHER /topic_name` means no node is publishing that topic.
- `NO_DATA /topic_name` means the monitor has not received a message yet.
- `STALE /topic_name age=...` means the topic exists but no new messages arrived within the stale timeout.
- `subscribers=0` on a publisher heartbeat means a node is publishing, but nobody is listening.
- `publishers=0` on a subscriber heartbeat means a node is waiting for input, but the upstream node is missing or on the wrong topic.

## Fast pipeline sanity check

For the real-camera vision pipeline, the healthy flow should look like:

```text
camera_node OUT /camera/image_raw rate ~= camera FPS
health_monitor IN /camera/image_raw fresh

yolo_node IN /camera/image_raw fresh
yolo_node OUT /detections fresh

tracker_node IN /detections fresh
tracker_node OUT /target_error fresh

control_node IN /target_error fresh
control_node IN /drone/telemetry fresh
control_node OUT /control_command fresh
```

If one stage is stale, fix the first broken upstream topic before chasing downstream symptoms.
