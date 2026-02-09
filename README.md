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

🗄️ archive/isaac-slam-nav2-attempt (Completed - Archived)
The Learning Experience

Attempted to integrate Isaac Sim with SLAM Toolbox for autonomous navigation
Got 95% working: LiDAR scans, ROS2 bridge, SLAM node configuration
Blocker: Isaac Sim's Transform Tree doesn't publish dynamic robot position
Value: Demonstrates 8+ hours of systematic debugging, problem-solving skills, and knowing when to pivot
All work documented in DEBUGGING_LOG.md and README_ISAAC_SLAM_ATTEMPT.md

🎯 feature/slam-gazebo (In Progress - HIGH PRIORITY)
Demo 1: Classical SLAM & Navigation

Uses Gazebo simulator with Turtlebot3 (reliable TF system)
Reuses all the SLAM/Nav2 configurations from the archived attempt
Goal: Working demo of real-time map building + autonomous navigation
Resume value: Industry-standard robotics demo showing ROS2, SLAM, path planning skills
Status: Ready to implement (2-3 hours estimated)

🤖 feature/isaac-cv (Planned)
Demo 2: Computer Vision Navigation

Keeps Isaac Sim (leverage its strengths: photorealistic rendering, synthetic data)
Pivots from SLAM to vision-based navigation
Adds camera + object detection (YOLO) for navigation decisions
Goal: Robot makes decisions based on visual input (approach objects, avoid others)
Resume value: Shows modern AI/ML skills, synthetic data generation for training
Status: Planned after slam-gazebo demo (2-3 hours estimated)

📊 main
Clean, polished branch with README linking to both working demos and the documented learning experience.

# RUN
/home/saman-aboutorab/isaacsim/python.sh src/main.py
/home/saman-aboutorab/isaacsim/python.sh src/00_basic_drive.py
/home/saman-aboutorab/isaacsim/python.sh src/20_ros2_laserscan.py

# ROS2 Topics
source /opt/ros/jazzy/setup.bash
ros2 topic list
ros2 topic hz /scan
ros2 topic echo /scan --once
ros2 topic hz /cmd_vel
ros2 topic echo /cmd_vel
ros2 topic echo /tf --once

# ROS2 cmd_vel manual
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}" 

# Before running the package
cd ~/projects/Robotics/isaac_diff_drive_nav/ros2_ws
source install/setup.bash
source /opt/ros/jazzy/setup.bash

# ROS2 nodes
ros2 run reactive_nav gap_follower
ros2 run reactive_nav scan_sanitizer
ros2 launch isaac_nav_bringup isaac_slam_nav.launch.py

#ROS2 updates on the node
cd ~/projects/Robotics/isaac_diff_drive_nav/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

#Rviz2
rviz2
ros2 param set /rviz use_sim_time true



