# TurtleBot3 Localization using EKF + AMCL (ROS 2 Humble)
## Overview
This project implements a complete localization pipeline for TurtleBot3 in ROS 2 Humble, combining:

* Local state estimation using Extended Kalman Filter (EKF)
* Global localization using Adaptive Monte Carlo Localization (AMCL)
* Gazebo simulation + RViz2 visualization

The system fuses:
* IMU data
* Wheel odometry
* 2D LiDAR scans
* Pre-built occupancy grid map

to achieve accurate and stable robot localization.

## 🧠 Localization Concept

<img width="826" height="553" alt="image" src="https://github.com/user-attachments/assets/3705c643-ddea-410a-81db-90d29ea13403" />

This project separates localization into two layers:

1️⃣ Local Localization — EKF (robot_localization)

The EKF fuses:

* /imu/data
* /wheel_odom

It produces:

* Filtered odometry → /odometry/filtered
* TF transform → odom → base_link

This provides smooth short-term motion estimation and reduces sensor noise.

2️⃣ Global Localization — AMCL (Nav2)

AMCL uses:

* /scan (Laser scan)
* /map (Map server)
* TF from EKF (odom → base_link)

It estimates:

* Robot pose in the map frame
* Publishes /amcl_pose
* Publishes TF → map → odom

This corrects long-term drift from wheel odometry.

## 📁 Launch Files

This project is organized into modular launch files to clearly separate
local estimation (EKF), map handling, and full localization execution.

---

### 1️⃣ `ekf_local.launch.py`

This launch file starts the **robot_localization EKF node** for local state estimation.

#### 🔹 Purpose
It performs **sensor fusion** between:
- IMU data (`/imu/data`)
- Wheel odometry (`/wheel_odom`)

#### 🔹 Output
- Filtered odometry: `/odometry/filtered`
- TF transform: `odom → base_link`

This provides smooth and drift-reduced short-term motion estimation,
which is required by AMCL for consistent global localization.

---

### 2️⃣ `map_server.launch.py`

AMCL requires a **static 2D occupancy grid map** for global localization.
This launch file starts the Nav2 map server to publish the saved map.

---

#### 🗺️ Creating the Map (SLAM)

Before running localization, a 2D map must be generated using SLAM.

For TurtleBot3 in simulation:

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```
Then teleoperate the robot to explore the environment and build the map.

Official TurtleBot3 SLAM documentation:

[Turtlebot3 emanual](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam_simulation/)


After mapping is complete, save the map:
```
ros2 run nav2_map_server map_saver_cli -f ekf_map
```
This generates:

* ekf_map.yaml
* ekf_map.pgm

Place these files inside the maps/ directory of this package.

🔹 What map_server.launch.py Starts

* nav2_map_server
* Publishes the /map topic
* Provides the occupancy grid required by AMCL

3️⃣ amcl_localization.launch.py

This is the main launch file of the project.

It integrates the complete localization pipeline and starts:

* Gazebo simulation world
* TurtleBot3 spawn
* EKF local node
* Map server
* AMCL
* Nav2 lifecycle manager
* RViz2

```
ros2 launch tb3_ekf_amcl_localization amcl_localization.launch.py
```
### After launching:

1) Set the initial pose in RViz2 using 2D Pose Estimate.
2) Open a new terminal and run teleoperation:

```
ros2 run turtlebot3_teleop teleop_keyboard
```
3) Observe localization performance in RViz2
