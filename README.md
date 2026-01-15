# Project Title (Isaac Sim + Python)
Isaac Differential Drive Navigation

## Goal
Built a physics-accurate mobile robot navigation system in NVIDIA Isaac Sim, implementing differential-drive kinematics, LiDAR-based obstacle avoidance, and PID wheel control with sensor noise modeling; validated behavior across randomized cluttered environments.

## Tech Stack
- NVIDIA Isaac Sim
- Python
- ROS2

## Project Structure
isaac_diff_drive_nav/
├── src/
│   ├── main.py
│   ├── nodes/
│   │   └── nav_demo.py
│   ├── robot/
│   │   └── diff_drive.py
│   ├── control/
│   │   └── avoidance.py
│   ├── sensors/
│   │   └── lidar.py
│   └── utils/
│       └── config.py
│
├── isaac/
│   ├── worlds/
│   │   └── simple_obstacles.usd
│   ├── robots/
│   │   └── diff_bot.usd
│   └── configs/
│       └── nav.yaml
│
├── README.md
└── requirements.txt

