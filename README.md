# QCar2 ROS 2 Autonomous Navigation

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

The current work in progress is obstacle avoidance using LiDAR and depth camera information.

The goal is to use both sensors to help the QCar2 detect obstacles around the vehicle and avoid them during navigation. The LiDAR can provide reliable distance measurements around the car, while the depth camera can give additional 3D information in front of the vehicle.

The following components are under active development:

- Obstacle detection using LiDAR data.
- Obstacle detection using camera depth data.
- Sensor-based decision making to help the vehicle avoid nearby obstacles.
- Integration of obstacle avoidance with the current navigation and control pipeline.

This part is intentionally marked as work in progress and is not the main focus of the current portfolio milestone.

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

## Trajectory Tracking

### 1. Model comparison for trajectory planning and NLMPC tracking

In this project, two vehicle models were tested in simulation to compare their behavior for trajectory planning and nonlinear MPC tracking:

- `QCar2_NLMPC`: a car-like kinematic model.
- `QCar2_NLMPC_SingleTrackModel`: a kinematic single-track model.

Both models were used to generate Bezier-based reference trajectories and then track those references using nonlinear MPC. The objective was to check whether the generated trajectory, the state tracking behavior, and the optimal control inputs were consistent between the two formulations.

After simulation and tuning, both models produced close tracking results. The single-track model was finally selected because its optimal input is more suitable for the low-level PID controller used on the QCar2 platform. This makes the connection between the high-level NLMPC output and the low-level vehicle controller more direct.

### 2. How to run the two simulation projects

The two simulation folders are:

```bash
QCar2_NLMPC
QCar2_NLMPC_SingleTrackModel
```

Both projects use the same general workflow:

```text
clean generated files -> build solver -> generate trajectory -> run NLMPC
```

All main parameters are configured in:

```bash
config/qcar2_nlmpc.yaml
```

This file contains the model parameters, constraints, MPC weights, acados settings, reference path settings, simulation initial condition, and output directory settings.

The cleaning step is important when configuration values are changed, especially values related to the solver or generated code. It removes the old generated acados files so that the next solver build is created from the updated configuration.

#### Run `QCar2_NLMPC`

```bash
cd QCar2_NLMPC
source setup_env.sh
python scripts/clean_generated.py
python scripts/build_solver.py
python scripts/generate_trajectory.py
python scripts/qcar2_nlmpc.py
```

The trajectory generation file for this project is:

```bash
QCar2_NLMPC/src/model_traj/flat_bezier_mk2.py
```

This is the file to edit if the waypoint list, heading boundary condition, segment time, or Bezier trajectory definition needs to be changed.

#### Run `QCar2_NLMPC_SingleTrackModel`

```bash
cd QCar2_NLMPC_SingleTrackModel
source setup_env.sh
python scripts/clean_generated.py
python scripts/build_solver.py
python scripts/generate_trajectory.py
python scripts/qcar2_nlmpc.py
```

The trajectory generation file for this project is:

```bash
QCar2_NLMPC_SingleTrackModel/src/model_traj/flat_bezier_single_track.py
```

This is the file to edit if the waypoint list, heading boundary condition, segment time, or Bezier trajectory definition needs to be changed.

#### When to rerun the scripts

If the solver-related configuration is changed, for example model parameters, constraints, MPC horizon, MPC weights, acados settings, generated solver directory, or generated simulation directory, run the full clean sequence again:

```bash
python scripts/clean_generated.py
python scripts/build_solver.py
python scripts/generate_trajectory.py
python scripts/qcar2_nlmpc.py
```

If the reference path settings inside `config/qcar2_nlmpc.yaml` are changed, it is also safer to run the full clean sequence:

```bash
python scripts/clean_generated.py
python scripts/build_solver.py
python scripts/generate_trajectory.py
python scripts/qcar2_nlmpc.py
```

If only the trajectory definition file is changed, for example waypoints, heading angles, segment times, or Bezier settings inside the trajectory file, regenerate the trajectory and run the NLMPC simulation again:

```bash
python scripts/generate_trajectory.py
python scripts/qcar2_nlmpc.py
```

If only the simulation initial state, initial input, run name, or maximum number of simulation steps is changed in `config/qcar2_nlmpc.yaml`, run only:

```bash
python scripts/qcar2_nlmpc.py
```

#### Generated reference and simulation results

For both projects, the generated reference trajectory is saved in:

```bash
data/reference/
```

This folder contains the generated reference CSV file and reference plots such as trajectory, heading, speed, steering angle, and other model-dependent variables.

The NLMPC tracking results are saved in:

```bash
results/runs/
```

Each simulation creates a run folder. The run folder contains files such as:

```bash
trajectory_tracking.png
states.png
controls.png
metrics.json
states.csv
controls.csv
reference_used.csv
reference_controls.csv
solve_times.csv
```

The most useful files for checking the result are `trajectory_tracking.png`, `states.png`, `controls.png`, and `metrics.json`.
