# Isaac Sim SLAM Integration Attempt

**Branch:** `archive/isaac-slam-nav2-attempt`
**Status:** ⚠️ 95% Complete — Blocked by Isaac Sim Transform Tree Issue
**Date:** February 2026

## Objective
Integrate NVIDIA Isaac Sim with ROS2 Jazzy SLAM Toolbox and Nav2 for autonomous mobile robot navigation with real-time map building.

## What Worked ✅

### 1. Isaac Sim → ROS2 Integration
- RTX LiDAR sensor publishing scans at 3 Hz to `/scan`
- ROS2 Clock publishing for simulation time synchronization
- Cmd_vel subscriber for robot control
- Robot responds to movement commands in simulation

### 2. SLAM Toolbox Configuration
- Node starts without lifecycle errors
- All parameters loaded correctly (odom_frame, base_frame, scan_topic)
- Subscribed to scan topic and receiving data
- Fixed double-activation lifecycle issues

### 3. Partial TF Chain
- `map → world` (published by SLAM)
- `chassis_link → lidar` (static transform)
- Proper simulation time handling across all nodes

## Blocking Issue ❌

**Isaac Sim's "ROS2 Publish Transform Tree" node does not publish dynamic TF transforms.**

Without the dynamic `world → chassis_link` transform showing robot movement:
- SLAM Toolbox sees robot as stationary
- No map building occurs
- Navigation stack cannot function

### Attempted Solutions
1. Verified Action Graph configuration (target prims, connections)
2. Checked Timeline is playing
3. Tried both static and dynamic transform approaches
4. Attempted workarounds with static TF publishers
5. Isaac Sim Transform Tree node remains non-functional

### Root Cause Analysis
- Likely Isaac Sim/Omniverse version compatibility issue with ROS2 Jazzy
- Possible USD file structure incompatibility
- May require Isaac Sim 4.x or a different OmniGraph setup

## Key Learnings

1. **Always verify TF chain first** before debugging higher-level systems
2. **Test with simpler simulators** (Gazebo) before complex ones (Isaac Sim)
3. **Isaac Sim requires additional setup** that may not be well-documented
4. **Lifecycle node debugging** requires checking both node state AND service availability

## Outcome

This experience directly informed the `feature/slam-gazebo` branch, which achieves
the same goal (SLAM + Nav2 autonomous navigation) using Gazebo — where TF publishing
works reliably out of the box. See [README_GAZEBO_SLAM.md](README_GAZEBO_SLAM.md)
for the working implementation.

## References

- [SLAM Toolbox Documentation](https://github.com/SteveMacenski/slam_toolbox)
- [Nav2 Documentation](https://navigation.ros.org/)
- [Isaac Sim ROS2 Bridge](https://docs.omniverse.nvidia.com/isaacsim/latest/ros2_tutorials/index.html)

---

**Conclusion:** This attempt demonstrated systematic debugging and problem-solving under
uncertainty. The 5% blocking issue (Isaac Sim TF) led to a strategic pivot to demonstrate
working systems in parallel branches — showing adaptability alongside technical depth.
