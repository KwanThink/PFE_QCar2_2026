# QCar2 ROS 2 Autonomous Navigation Workspace

> **Résumé en français**  
> Ce dépôt présente un workspace ROS 2 développé pour la plateforme physique Quanser QCar2.  
> Il inclut l’intégration matérielle, l’odométrie, le LiDAR, la cartographie SLAM avec Cartographer, la navigation avec Nav2, ainsi que la génération et la visualisation de trajectoires de type Bézier dans RViz.  
> La partie contrôle avancé par NLMPC avec solveur ACADO/acados est actuellement en cours de développement.

---

## Overview

This repository contains a ROS 2 workspace for the **Quanser QCar2 physical platform**, developed and tested on an **NVIDIA Jetson Orin**.

The current implemented milestone focuses on:

- QCar2 hardware and sensor integration
- Odometry and TF setup
- LiDAR-based SLAM using Cartographer
- Navigation bringup using Nav2
- Bezier-based trajectory generation
- Trajectory visualization in RViz

The project is structured as a ROS 2 workspace with custom nodes and interfaces for QCar2.

---

## System Environment

The project was developed on the following setup:

```bash
Ubuntu 20.04
ROS 2 Humble
NVIDIA Jetson Orin
Quanser QCar2 physical platform
