# Isaac Sim Differential Drive Navigation

Built a physics-accurate mobile robot navigation system in NVIDIA Isaac Sim, implementing differential-drive kinematics, LiDAR-based obstacle avoidance, and ROS2 integration.

> **Note:** The Gazebo-based SLAM, Nav2, and vision navigation work has moved to a dedicated repo:
> [gazebo_robot_nav](https://github.com/saman-aboutorab/gazebo_robot_nav)

## Tech Stack

- NVIDIA Isaac Sim
- Python
- ROS2 Jazzy

## Branches

### `main` — Baseline Isaac Sim Drive
Core differential drive robot with LiDAR-based obstacle avoidance in Isaac Sim.

### `features/rtx-lidar-pointcloud` — RTX LiDAR Pipeline
RTX-accelerated LiDAR pointcloud processing, wall-following math, and velocity control based on sensor readings.

### `archive/isaac-slam-nav2-attempt` — SLAM Integration (Archived)
Attempted full SLAM Toolbox + Nav2 integration with Isaac Sim. Got 95% working (LiDAR scans, ROS2 bridge, SLAM node config). Blocked by Isaac Sim's Transform Tree not publishing dynamic robot positions. Documented extensively in `DEBUGGING_LOG.md` and `README_ISAAC_SLAM_ATTEMPT.md`.

### `feature/isaac-cv` — Computer Vision Navigation (Planned)
Roadmap for vision-based navigation using Isaac Sim's photorealistic rendering and synthetic data generation with YOLO object detection.

## Run

```bash
/home/saman-aboutorab/isaacsim/python.sh src/00_basic_drive.py
```
