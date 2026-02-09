# Isaac Sim SLAM Integration Attempt

**Branch:** `archive/isaac-slam-nav2-attempt`
**Status:** ⚠️ 95% Complete - Blocked by Isaac Sim Transform Tree Issue
**Date:** February 2026

## Objective
Integrate NVIDIA Isaac Sim with ROS2 Jazzy SLAM Toolbox and Nav2 for autonomous mobile robot navigation with real-time map building.

## What Works ✅

### 1. Isaac Sim → ROS2 Integration
- ✅ RTX LiDAR sensor publishing scans at 3 Hz to `/scan`
- ✅ ROS2 Clock publishing for simulation time synchronization
- ✅ Cmd_vel subscriber for robot control
- ✅ Robot responds to movement commands in simulation

### 2. SLAM Toolbox Configuration
- ✅ Node starts without lifecycle errors
- ✅ All parameters loaded correctly (odom_frame, base_frame, scan_topic)
- ✅ Subscribed to scan topic and receiving data
- ✅ Fixed double-activation lifecycle issues

### 3. Partial TF Chain
- ✅ `map → world` (published by SLAM)
- ✅ `chassis_link → lidar` (static transform)
- ✅ Proper simulation time handling across all nodes

### 4. Configuration Files
- ✅ `slam_params.yaml` - Complete SLAM Toolbox configuration
- ✅ `nav2_params.yaml` - Full Nav2 stack parameters
- ✅ Launch files for integrated stack

## Blocking Issue ❌

**Isaac Sim's "ROS2 Publish Transform Tree" node does not publish dynamic TF transforms.**

### Impact
Without the dynamic `world → chassis_link` transform showing robot movement:
- SLAM Toolbox sees robot as stationary
- No map building occurs (map topic exists but publishes no data)
- Navigation stack cannot function

### Attempted Solutions
1. ✅ Verified Action Graph configuration (target prims, connections)
2. ✅ Checked Timeline is playing
3. ✅ Tried both static and dynamic transform approaches
4. ✅ Attempted workarounds with static TF publishers
5. ❌ Isaac Sim Transform Tree node remains non-functional

### Root Cause Analysis
- Likely Isaac Sim/Omniverse version compatibility issue with ROS2 Jazzy
- Possible USD file structure incompatibility
- May require Isaac Sim 4.x or different setup approach

## Key Learnings

### Technical Skills Demonstrated
- ROS2 Jazzy lifecycle node management
- SLAM Toolbox configuration and debugging
- TF tree debugging and visualization
- Simulation time synchronization
- Nav2 stack configuration
- Complex multi-node system integration

### Problem-Solving Process
1. Systematic debugging (8+ hours documented in DEBUGGING_LOG.md)
2. Root cause analysis using ROS2 introspection tools
3. Workaround attempts when blocked
4. Clear documentation of issues and solutions

## Files Created

### Configuration
- `ros2_ws/src/isaac_nav_bringup/config/slam_params.yaml`
- `ros2_ws/src/isaac_nav_bringup/config/nav2_params.yaml`

### Launch Files
- `ros2_ws/src/isaac_nav_bringup/launch/isaac_slam_nav.launch.py`
- `ros2_ws/src/isaac_nav_bringup/launch/navigation_launch_no_dock.py`

### Documentation
- `DEBUGGING_LOG.md` - Comprehensive debugging timeline
- `.claude/projects/.../memory/MEMORY.md` - Key learnings

### Diagnostic Tools
- `test_slam_alone.sh` - Test SLAM Toolbox in isolation
- `ros2_ws/check_sim_time.sh` - Verify sim_time configuration

## Next Steps (Other Branches)

This branch serves as the foundation. The working components will be reused:

### Branch: `feature/slam-gazebo`
- Use Gazebo simulator (TF works out-of-box)
- Reuse SLAM and Nav2 configurations
- Demonstrate successful autonomous mapping

### Branch: `feature/isaac-cv`
- Keep Isaac Sim integration
- Focus on computer vision instead of SLAM
- Leverage Isaac Sim's synthetic data generation

## Running This Branch

### Prerequisites
```bash
# Isaac Sim installed at ~/isaacsim/
# ROS2 Jazzy installed
source /opt/ros/jazzy/setup.bash
```

### Launch
```bash
# Terminal 1: Isaac Sim
/home/saman-aboutorab/isaacsim/python.sh src/20_ros2_laserscan.py

# Terminal 2: SLAM + Nav2 Stack
cd ros2_ws
source install/setup.bash
ros2 launch isaac_nav_bringup isaac_slam_nav.launch.py

# Terminal 3: RViz
rviz2
```

### Expected Behavior
- Isaac Sim loads with Nova Carter robot
- SLAM Toolbox starts (active state)
- RViz shows laser scans
- Robot responds to cmd_vel
- ⚠️ Map does not build (TF issue)

## Diagnostic Commands

```bash
# Check TF transforms
ros2 run tf2_tools view_frames

# Check SLAM state
ros2 lifecycle get /slam_toolbox

# Verify parameters
ros2 param get /slam_toolbox base_frame

# Check topics
ros2 topic hz /scan --use-sim-time
ros2 topic hz /map --use-sim-time
```

## Lessons for Future Work

1. **Always verify TF chain first** before debugging higher-level systems
2. **Test with simpler simulators** (Gazebo) before complex ones (Isaac Sim)
3. **Isaac Sim requires additional setup** that may not be well-documented
4. **Static transforms can mask dynamic TF issues** - use carefully
5. **Lifecycle node debugging** requires checking both node state AND service availability

## References

- [SLAM Toolbox Documentation](https://github.com/SteveMacenski/slam_toolbox)
- [Nav2 Documentation](https://navigation.ros.org/)
- [Isaac Sim ROS2 Bridge](https://docs.omniverse.nvidia.com/isaacsim/latest/ros2_tutorials/index.html)
- TF Debugging: `ros2 run tf2_ros tf2_echo <source> <target>`

---

**Conclusion:** This attempt demonstrated advanced ROS2 skills and systematic debugging. The 5% blocking issue (Isaac Sim TF) led to a strategic pivot to demonstrate working systems in parallel branches.
