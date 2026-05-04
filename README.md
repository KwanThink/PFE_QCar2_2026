# QCar2 ROS 2 Autonomous Navigation Workspace

## Overview

This repository contains a ROS 2 workspace for the **Quanser QCar2 physical platform**, developed and tested on an **NVIDIA Jetson Orin**. The project focuses on building an autonomous navigation pipeline for QCar2, starting from hardware and sensor integration, then progressing toward SLAM, Nav2-based navigation, and trajectory generation. In the current completed milestone, I explored the QCar2 platform, tested the onboard camera and LiDAR, collected sensor data, and visualized them through a remote development setup using SSH. I also applied SLAM and Nav2 to scan the environment, generate a map, and navigate inside the mapped area using a PID-based control approach. In addition, I used the QCar2 kinematic model to generate trajectories that better match the vehicle motion constraints instead of relying only on the default Nav2-generated trajectory, and visualized the generated trajectories in RViz. The next stage of the project is focused on NLMPC trajectory tracking, ACADO/acados solver integration, and closed-loop optimal control for trajectory following.

---

## Hardware Setup

The system runs on an **NVIDIA Jetson Orin onboard computer** mounted on the **Quanser QCar2 physical platform**.

The Jetson Orin acts as the onboard computer. It runs Ubuntu, ROS 2, and all ROS 2 nodes for:

- Hardware integration
- Sensor drivers
- Odometry
- SLAM
- Navigation
- Trajectory generation
- Command conversion between Nav2 and QCar2 hardware

---

## System Environment

The project was developed on the following setup:

```bash
Ubuntu 20.04
ROS 2 Humble
NVIDIA Jetson Orin
Quanser QCar2 physical platform
```

---

## Build Instructions

From the ROS 2 workspace directory:

```bash
cd ~/ros2
colcon build
source install/setup.bash
```

---

## Representative Launch Files

This section summarizes the main launch files used in the current project milestone.

### 1. SLAM and Navigation Bringup

```bash
ros2 launch qcar2_nodes qcar2_slam_and_nav_bringup_launch.py
```

This is the main launch file for the QCar2 SLAM and navigation pipeline.

It starts the complete bringup needed for autonomous navigation, including:

- QCar2 base bringup
- Cartographer SLAM
- Occupancy grid generation
- Nav2 navigation stack
- Nav2-to-QCar2 command converter

Internally, this launch file includes the Cartographer launch file, so Cartographer does not need to be launched separately.

This launch file is used when the QCar2 needs to perform SLAM/navigation with the physical robot setup.

### 2. Cartographer SLAM Launch

```bash
ros2 launch qcar2_nodes qcar2_cartographer_launch.py
```

This launch file starts the SLAM-related components using Cartographer.

It includes:

- QCar2 base launch
- LiDAR frame TF setup
- Cartographer SLAM node
- Cartographer occupancy grid node

In normal usage, this file is already included by:

```bash
ros2 launch qcar2_nodes qcar2_slam_and_nav_bringup_launch.py
```

Therefore, it is mainly useful for testing or debugging the SLAM part independently.

### 3. Bezier Trajectory Visualization

```bash
ros2 launch qcar2_nodes qcar2_bezier_launch.py
```

This launch file is used for the current trajectory generation milestone.

It starts:

- Nav2 map server
- Nav2 lifecycle manager
- QCar2 Bezier trajectory node

The Bezier node loads trajectory parameters from:

```bash
qcar2_nodes/config/qcar2_trajectory.yaml
```

The default map path is configured for the physical QCar2/Jetson environment:

```bash
/home/nvidia/Maps/d215_map.yaml
```

This map is stored in the QCar2 runtime environment.

To use another map, override the launch argument:

```bash
ros2 launch qcar2_nodes qcar2_bezier_launch.py map_yaml:=/absolute/path/to/map.yaml
```

This launch file is used to visualize the planned Bezier trajectory in RViz before closed-loop trajectory tracking is enabled.

### 4. Full Localization and Navigation Launch

```bash
ros2 launch qcar2_nodes qcar2_localize_and_nav_launch.py
```

This launch file brings up the localization and navigation pipeline using an existing map.

It includes:

- QCar2 hardware node
- QCar2 odometry node
- LiDAR node
- Fixed LiDAR TF node
- Nav2 map server
- AMCL localization
- Nav2 navigation stack
- Nav2-to-QCar2 command converter
- Optional RViz visualization

This launch file is useful when a map already exists and the robot should localize itself and navigate within that known environment.

---

## Main Demo Commands

For the current portfolio milestone, the two most important launch files are:

```bash
ros2 launch qcar2_nodes qcar2_slam_and_nav_bringup_launch.py
```

```bash
ros2 launch qcar2_nodes qcar2_bezier_launch.py
```

---

## Trajectory Generation

The current trajectory module generates and visualizes a Bezier-based trajectory for QCar2.

The trajectory configuration is stored in:

```bash
ros2/src/qcar2_nodes/config/qcar2_trajectory.yaml
```

The corresponding node is:

```bash
ros2/src/qcar2_nodes/src/qcar2_bezier.cpp
```

The node is launched with:

```bash
ros2 launch qcar2_nodes qcar2_bezier_launch.py
```

This milestone demonstrates the trajectory generation and visualization stage before closed-loop trajectory tracking.

---

## SLAM and Navigation

The SLAM and navigation pipeline is launched using:

```bash
ros2 launch qcar2_nodes qcar2_slam_and_nav_bringup_launch.py
```

This launch file combines the Cartographer-based SLAM setup with the Nav2 navigation stack.

The Cartographer configuration is defined in:

```bash
ros2/src/qcar2_nodes/config/qcar2_2d.lua
```

The Nav2 configuration is defined in:

```bash
ros2/src/qcar2_nodes/config/qcar2_slam_and_nav.yaml
```

This setup is intended for running SLAM and navigation on the physical QCar2 platform using the Jetson Orin onboard computer.

---

## Navigation Command Conversion

Nav2 publishes velocity commands using standard ROS 2 messages such as `geometry_msgs/Twist`.

The QCar2 platform requires its own motor command format. The node below acts as an adapter between Nav2 and the QCar2 hardware interface:

```bash
ros2/src/qcar2_nodes/src/nav2_qcar_command_convert.cpp
```

It converts Nav2 velocity commands into QCar2-specific motor commands for throttle and steering.

---

## Project Goal

The goal of this project is to develop a ROS 2-based autonomous navigation pipeline for the Quanser QCar2 platform.

The project starts from:

- Hardware integration
- Sensor integration
- Odometry
- SLAM
- Navigation bringup

It then progresses toward:

- Trajectory generation
- Bezier trajectory visualization
- Closed-loop trajectory tracking
- NLMPC-based optimal control

The current milestone demonstrates the system foundation and trajectory generation pipeline, while the next stage focuses on NLMPC-based trajectory tracking using ACADO/acados.

---

## Author

Developed as part of the PFE QCar2 autonomous navigation project.
