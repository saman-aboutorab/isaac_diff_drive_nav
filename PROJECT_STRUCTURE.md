# Project Structure & Resume Strategy

**Repository:** `isaac_diff_drive_nav`
**Focus:** Autonomous Mobile Robotics with ROS2 Jazzy
**Target Roles:** Software Engineer / AI Engineer in Robotics

## Git Branch Strategy

```
main (stable demos)
│
├── feature/ros2-laserscan ✅ COMPLETE
│   └── "Learning Experience" - Documented failed attempt
│       - Shows debugging skills
│       - Shows problem-solving process
│       - 95% functional but blocked by simulator issue
│
├── feature/slam-gazebo 🚧 IN PROGRESS
│   └── Demo 1: "ROS2 SLAM & Autonomous Navigation"
│       - Gazebo + Turtlebot3 + SLAM + Nav2
│       - Real-time map building
│       - Autonomous path planning
│       - Industry-standard demo
│
└── feature/isaac-cv 🚧 PLANNED
    └── Demo 2: "Isaac Sim Computer Vision Pipeline"
        - Object detection in simulation
        - Synthetic data generation
        - Vision-based navigation
        - Modern AI/ML approach
```

## Repository Structure

```
isaac_diff_drive_nav/
├── isaac/
│   ├── worlds/
│   │   ├── simple_obstacles_carter_lidar.usd    # Isaac Sim world
│   │   └── objects_scene.usd                    # CV demo world (future)
│   └── action_graphs/                           # Isaac Sim graphs
│
├── ros2_ws/
│   ├── reactive_nav/                            # Reactive navigation package
│   │   ├── gap_follower.py                      # ✅ Works! 3-sector avoidance
│   │   └── scan_sanitizer.py                    # LiDAR data cleaner
│   │
│   ├── src/isaac_nav_bringup/                   # Main navigation package
│   │   ├── launch/
│   │   │   ├── isaac_slam_nav.launch.py         # Isaac SLAM attempt
│   │   │   ├── gazebo_slam.launch.py            # 🚧 Gazebo demo
│   │   │   └── navigation_launch_no_dock.py     # Nav2 nodes
│   │   └── config/
│   │       ├── slam_params.yaml                 # ✅ Complete
│   │       └── nav2_params.yaml                 # ✅ Complete
│   │
│   └── vision_nav/                              # 🚧 CV demo package (future)
│       ├── object_detector.py
│       └── cv_gap_follower.py
│
├── src/
│   ├── 00_basic_drive.py                        # Basic Isaac Sim control
│   └── 20_ros2_laserscan.py                     # Isaac ROS2 bridge
│
└── docs/
    ├── README_ISAAC_SLAM_ATTEMPT.md             # ✅ Failure analysis
    ├── DEBUGGING_LOG.md                         # ✅ 8+ hours documented
    ├── GAZEBO_SLAM_ROADMAP.md                   # 🚧 Implementation plan
    ├── ISAAC_CV_ROADMAP.md                      # 🚧 Implementation plan
    └── PROJECT_STRUCTURE.md                     # ← You are here
```

## Resume Presentation Strategy

### Portfolio Website / GitHub README

#### Project Title
**"Multi-Modal Autonomous Navigation: SLAM & Computer Vision Demos"**

#### Project Description
> Developed two complementary demonstrations of autonomous mobile robot navigation using ROS2 Jazzy, showcasing both classical SLAM techniques and modern computer vision approaches.
>
> **Demo 1** implements real-time SLAM and autonomous navigation using Gazebo simulation. **Demo 2** leverages NVIDIA Isaac Sim for synthetic data generation and vision-based navigation.
>
> The project demonstrates proficiency in ROS2, sensor fusion, path planning, and modern robotics simulation tools.

### Talking Points by Demo

#### Demo 1: Gazebo SLAM (feature/slam-gazebo)
**"Classical Robotics Approach"**

**What I Built:**
- Configured SLAM Toolbox for real-time 2D mapping
- Integrated Nav2 stack with custom costmap parameters
- Implemented sensor fusion (LiDAR + odometry)
- Developed launch file infrastructure for multi-node systems

**Technical Skills:**
- ROS2 Jazzy (nodes, topics, services, parameters, TF)
- SLAM algorithms and configuration
- Navigation stack (global/local planners, costmaps)
- Gazebo simulation

**Key Result:** Robot autonomously explores, builds map, and navigates to goals

#### Demo 2: Isaac Sim CV (feature/isaac-cv)
**"Modern AI/ML Approach"**

**What I Built:**
- Integrated NVIDIA Isaac Sim with ROS2
- Implemented real-time object detection pipeline
- Developed behavior-based navigation using vision
- Created synthetic training data generation system

**Technical Skills:**
- NVIDIA Isaac Sim / Omniverse
- Computer Vision (YOLO, OpenCV, TensorRT)
- Synthetic data generation for ML training
- Multi-sensor fusion (Camera + LiDAR)

**Key Result:** Robot makes navigation decisions based on visual input

#### Bonus: Learning from Failure (feature/ros2-laserscan)
**"Problem-Solving & Debugging"**

**What I Attempted:**
- Tried integrating Isaac Sim directly with SLAM Toolbox
- Successfully configured 95% of system
- Identified root cause: Isaac Sim Transform Tree bug
- Pivoted strategy when blocked

**What This Shows:**
- Systematic debugging skills (8+ hours documented)
- Root cause analysis ability
- Adaptability when facing blockers
- Clear technical communication

## Interview Talking Points

### "Tell me about a challenging technical problem"
> "I was integrating NVIDIA Isaac Sim with ROS2's SLAM system. Everything worked except the map wouldn't build. After systematic debugging, I discovered Isaac Sim's Transform Tree wasn't publishing the robot's dynamic position - the robot appeared frozen to SLAM even though it was moving. Rather than spending weeks on a simulator bug, I pivoted to two parallel demos: one proving my SLAM skills in Gazebo, another leveraging Isaac Sim's strengths in computer vision. This showed me the importance of knowing when to persist vs. when to pivot."

### "What's your experience with ROS?"
> "I've worked extensively with ROS2 Jazzy. I built a complete navigation system integrating SLAM Toolbox, Nav2 stack, and custom navigation behaviors. I'm comfortable with the full ROS2 stack - nodes, topics, services, actions, parameters, TF transformations, and lifecycle management. I've also written launch files, parameter configurations, and debugged complex multi-node systems."

### "Any experience with simulation?"
> "Yes, I've worked with both Gazebo and NVIDIA Isaac Sim. I built a SLAM demo in Gazebo and integrated Isaac Sim with ROS2 for computer vision work. I understand the tradeoffs - Gazebo is great for standard robotics testing, while Isaac Sim excels at photorealistic rendering and synthetic data generation for ML training."

### "Computer vision experience?"
> "In my Isaac Sim project, I implemented an object detection pipeline using YOLO and integrated it with robot navigation. The robot made real-time decisions based on visual input - like approaching certain objects while avoiding others. I also explored synthetic data generation for training vision models, which is valuable for scenarios where real-world data is expensive or dangerous to collect."

## GitHub Repository Best Practices

### README.md (Main)
```markdown
# Autonomous Mobile Robot Navigation Demos

Two complementary demonstrations of mobile robot autonomy:

**[Demo 1: SLAM & Navigation (Gazebo)](tree/feature/slam-gazebo)**
- Real-time mapping with SLAM Toolbox
- Autonomous navigation with Nav2
- [Video Demo] [Documentation]

**[Demo 2: Computer Vision Navigation (Isaac Sim)](tree/feature/isaac-cv)**
- Object detection pipeline
- Vision-based navigation
- Synthetic data generation
- [Video Demo] [Documentation]

**Tech Stack:** ROS2 Jazzy, Gazebo, Isaac Sim, SLAM Toolbox, Nav2, YOLO
```

### Branch READMEs
- Each branch has detailed README with:
  - Quick start instructions
  - Architecture diagram
  - Configuration notes
  - Demo video/screenshots

### Documentation
- Well-commented code
- Clear commit messages
- Technical decisions documented
- Learning process visible (feature/ros2-laserscan)

## Timeline

| Branch | Status | Time Required | Priority |
|--------|--------|---------------|----------|
| ros2-laserscan | ✅ Complete | - | Archive |
| slam-gazebo | 🚧 In Progress | 2-3 hours | **HIGH** |
| isaac-cv | 📋 Planned | 2-3 hours | Medium |
| main | 📝 Documentation | 1 hour | After demos |

**Total Estimate:** 5-7 hours to complete both working demos

## Next Actions

1. **Immediate:** Implement Gazebo SLAM demo (highest resume value)
2. **Next:** Record demo video and take screenshots
3. **Then:** Implement Isaac CV demo
4. **Finally:** Create polished main README with both demos
5. **Polish:** Record short video explaining project structure

---

**This structure tells a compelling story:**
1. Attempted hard technical challenge (SLAM + Isaac Sim)
2. Demonstrated debugging/problem-solving
3. Pivoted strategically
4. Delivered TWO working demos showing breadth
5. Clear documentation throughout

**Perfect for resume/portfolio!** 🎯
