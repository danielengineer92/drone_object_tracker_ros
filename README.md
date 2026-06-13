# Drone Vision System

A ROS 2 Jazzy-based vision-guided drone system for Raspberry Pi 4 with PX4 flight controller.

## Overview

This system provides:
- Real-time camera capture
- YOLO object detection (Ultralytics)
- Target tracking with error calculation
- PX4 telemetry via MAVSDK
- Control command generation with safety gating
- Real-time visualization with diagnostics

## Safety

**CRITICAL**: The `autonomous_enabled` parameter defaults to `false`.
When disabled:
- No flight commands are sent to the aircraft
- All intended commands are logged for analysis
- The system operates in observation-only mode

To enable autonomous flight (use at your own risk):
```bash
ros2 param set /control_node autonomous_enabled true

Hardware Requirements
Raspberry Pi 4 (4GB+ recommended)
Ubuntu 24.04
USB Camera or Raspberry Pi Camera
Pixhawk 6C with PX4 firmware
Serial connection between Pi and Pixhawk
Software Requirements
ROS 2 Jazzy
Python 3.12+
OpenCV 4.8+
Ultralytics 8.0+
MAVSDK 1.4+
Installation
mkdir -p ~/drone_ws/src
cd ~/drone_ws/src
# Copy all packages here
2. Install system dependencies
sudo apt update
sudo apt install -y \
  ros-jazzy-cv-bridge \
  ros-jazzy-sensor-msgs \
  python3-pip \
  python3-opencv \
  libopencv-dev
3. Install Python dependencies
cd ~/drone_ws
pip install -r requirements.txt
4. Build the workspace
cd ~/drone_ws
colcon build --symlink-install
source install/setup.bash
5. Download YOLO model
# The model will be auto-downloaded on first run, or pre-download:
python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"
Usage
Simulation Mode (No hardware needed)
source ~/drone_ws/install/setup.bash
ros2 launch drone_bringup simulation_launch.py
Vision Mode (Camera only, no drone)
source ~/drone_ws/install/setup.bash
ros2 launch drone_bringup vision_launch.py
With options:
ros2 launch drone_bringup vision_launch.py camera_index:=0 model_path:=yolov8n.pt target_class:=person
Full System Mode (Camera + Drone)
source ~/drone_ws/install/setup.bash
ros2 launch drone_bringup full_system_launch.py connection_url:="serial:///dev/ttyACM0:57600"

┌──────────────┐     ┌──────────────┐     ┌──────────────┐
│  camera_node │────▶│  yolo_node   │────▶│ tracker_node │
│              │     │              │     │              │
│ /camera/     │     │ /detections  │     │ /target_     │
│  image_raw   │     │              │     │  error       │
└──────────────┘     └──────────────┘     └──────┬───────┘
                                                  │
┌──────────────┐                           ┌──────▼───────┐
│telemetry_node│──────────────────────────▶│ control_node │
│              │                           │              │
│ /drone/      │                           │ /control_    │
│  telemetry   │                           │  command     │
└──────────────┘                           └──────────────┘

┌──────────────────────────────────────────────────────────┐
│                    visualizer_node                         │
│  Subscribes to all topics for real-time display           │
└──────────────────────────────────────────────────────────┘


Monitoring
View topics
ros2 topic list
ros2 topic echo /target_error
ros2 topic echo /control_command
ros2 topic echo /drone/telemetry

Check parameters
ros2 param list /control_node
ros2 param get /control_node autonomous_enabled

Node status
ros2 node list
ros2 node info /control_node


## Quick Start

```bash
# 1. Set up workspace
mkdir -p ~/drone_ws/src
cd ~/drone_ws/src
# Place all packages in src/

# 2. Install deps
pip install -r ~/drone_ws/requirements.txt
sudo apt install ros-jazzy-cv-bridge

# 3. Build
cd ~/drone_ws
colcon build --symlink-install
source install/setup.bash

# 4. Run simulation (no hardware needed)
ros2 launch drone_bringup simulation_launch.py

drone_ws/
├── requirements.txt
├── README.md
└── src/
├── drone_interfaces/
│ ├── package.xml
│ ├── CMakeLists.txt
│ └── msg/
│ ├── Detection.msg
│ ├── DetectionArray.msg
│ ├── TargetError.msg
│ ├── DroneTelemetry.msg
│ └── ControlCommand.msg
├── drone_camera/
│ ├── package.xml
│ ├── setup.py
│ ├── setup.cfg
│ ├── resource/drone_camera
│ └── drone_camera/
│ ├── init.py
│ └── camera_node.py
├── drone_yolo/
│ ├── package.xml
│ ├── setup.py
│ ├── setup.cfg
│ ├── resource/drone_yolo
│ └── drone_yolo/
│ ├── init.py
│ └── yolo_node.py
├── drone_tracker/
│ ├── package.xml
│ ├── setup.py
│ ├── setup.cfg
│ ├── resource/drone_tracker
│ └── drone_tracker/
│ ├── init.py
│ └── tracker_node.py
├── drone_telemetry/
│ ├── package.xml
│ ├── setup.py
│ ├── setup.cfg
│ ├── resource/drone_telemetry
│ └── drone_telemetry/
│ ├── init.py
│ └── telemetry_node.py
├── drone_control/
│ ├── package.xml
│ ├── setup.py
│ ├── setup.cfg
│ ├── resource/drone_control
│ └── drone_control/
│ ├── init.py
│ └── control_node.py
├── drone_visualizer/
│ ├── package.xml
│ ├── setup.py
│ ├── setup.cfg
│ ├── resource/drone_visualizer
│ └── drone_visualizer/
│ ├── init.py
│ └── visualizer_node.py
├── drone_fake/
│ ├── package.xml
│ ├── setup.py
│ ├── setup.cfg
│ ├── resource/drone_fake
│ └── drone_fake/
│ ├── init.py
│ ├── fake_camera_node.py
│ ├── fake_detection_node.py
│ └── fake_telemetry_node.py
└── drone_bringup/
├── package.xml
├── CMakeLists.txt
├── config/
│ ├── simulation_params.yaml
│ ├── vision_params.yaml
│ └── full_system_params.yaml
└── launch/
├── simulation_launch.py
├── vision_launch.py
└── full_system_launch.py