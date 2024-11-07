# robot_dashboard

[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](
https://opensource.org/licenses/BSD-3-Clause)
[![Ros1 Version](https://img.shields.io/badge/ROS-Noetic-green)](
http://wiki.ros.org/noetic)
[![Ros2 Version](https://img.shields.io/badge/ROS-Humble-green)](
https://docs.ros.org/en/humble/index.html)

Dashboard interface for monitoring the robots of the team.

# Get Started

## Installation

### From Docker
The easiest way to get started is to use the provided Docker image. You can find the Dockerfile in the `docker` folder for ROS and ROS2.

To build the ROS image, run the following command in the ros1 folder:
```bash
docker build -t dashboard_ros1 .
```

To build the ROS2 image, run the following command in the ros2 folder:
```bash
docker build -t dashboard_ros2 .
```

# Usage
 Once you have both images built, you can run the following command to start the dashboard for ROS1:

```bash
sh run_ros1.sh python3 <python_file> configs/<custom_config_file> layouts/<custom_layout_file>.yaml
```

To start the dashboard for ROS2, run the following command:

```bash
sh run_ros2.sh python3 <python_file> configs/<custom_config_file> layouts/<custom_layout_file>.yaml
```

To start the insta360 camera controller, run the following command:

```bash
sh run_ros2.sh python3 insta360_controller.py
```

## Python Files
- `dashboard_robot.py`: Dashboard for monitoring the robot's status. **Only working for ROS1.**
- `dashboard_video.py`: Dashboard for monitoring the robot's video feed. **Only working for ROS2.**
- `dashboard_wifi_monitor.py`: Dashboard for monitoring the robot's wifi connection. **Only working for ROS2.**
- `insta360_controller.py`: Controller for the Insta360 camera. **Only working for ROS2.**


## Custom Config File
- `configs/config.yaml`: General configuration file for the dashboard.
- `layout/layout.yaml`: Layout configuration file to display the widgets of the dashboard.

# Photo
![Screenshot](misc/screenshot.png?raw=true "Screenshot")
