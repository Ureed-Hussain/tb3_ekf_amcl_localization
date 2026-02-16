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


