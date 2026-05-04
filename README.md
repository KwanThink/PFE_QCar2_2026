# QCar2 ROS 2 Autonomous Navigation Workspace

## Overview

This repository contains a ROS 2 workspace for the **Quanser QCar2 physical platform**, developed and tested on an **NVIDIA Jetson Orin** onboard computer. The project goal is: **Control Implementation for a Safe and Secure Self-driving Car**.

---

## Current Completed Milestone

The current GitHub version focuses on the completed system exploration, SLAM/Nav2 bringup, and trajectory generation milestones.

Implemented:

- Explored the Quanser QCar2 platform, tested the onboard camera and LiDAR, collected sensor data, and visualized them through a remote development setup using SSH.
- Applied SLAM and Nav2 to scan the environment, generate a map, and navigate inside the mapped area using a PID-based control approach.
- Used the QCar2 kinematic model to generate trajectories that better match the vehicle motion constraints instead of relying only on the default Nav2-generated trajectory, and visualized the generated trajectories in RViz.
- The scanned map used for localization and navigation can be found in the `Maps/` directory.

---

## Work in Progress

The following components are under active development:

- NLMPC trajectory tracking
- ACADO/acados solver integration
- Closed-loop optimal control for trajectory following

These parts are intentionally marked as work in progress and are not the main focus of the current portfolio milestone.

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
## Representative Launch Files

This section summarizes the main launch files used in the current project milestone.

### 1. SLAM and Navigation Bringup

```bash
ros2 launch qcar2_nodes qcar2_slam_and_nav_bringup_launch.py
```

Main launch file for running the QCar2 SLAM and navigation pipeline.  
It brings up the QCar2 platform, Cartographer SLAM, Nav2 navigation, and the Nav2-to-QCar2 command converter.

### 2. Cartographer SLAM Launch

```bash
ros2 launch qcar2_nodes qcar2_cartographer_launch.py
```

Launch file for running the Cartographer SLAM pipeline independently.  
It is mainly used for mapping, occupancy grid generation, and SLAM debugging.

### 3. Bezier Trajectory Visualization

```bash
ros2 launch qcar2_nodes qcar2_bezier_launch.py
```

Launch file for generating and visualizing the QCar2 Bezier trajectory in RViz.  
It uses the trajectory configuration from `qcar2_nodes/config/qcar2_trajectory.yaml` and the map stored in the QCar2 runtime environment.

### 4. Full Localization and Navigation Launch

```bash
ros2 launch qcar2_nodes qcar2_localize_and_nav_launch.py
```

Launch file for localization and navigation using an existing map.  
It starts the QCar2 hardware, odometry, LiDAR, AMCL localization, Nav2 navigation, and optional RViz visualization.

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
