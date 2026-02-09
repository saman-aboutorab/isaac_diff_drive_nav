# Isaac Sim + ROS2 SLAM Integration Debugging Log
**Date:** 2026-02-08
**Goal:** Get SLAM Toolbox working with Isaac Sim + ROS2 Jazzy + Nav2

## Final Status
- ✅ Isaac Sim publishing scans at ~3 Hz
- ✅ ROS2 bridge functional (clock, TF, cmd_vel)
- ✅ Static TF publishers: world → chassis_link → lidar
- ✅ SLAM Toolbox node active with correct parameters
- ✅ No lifecycle errors
- ❌ **Map not publishing** (root cause unknown)

## Issues Solved

### 1. TF Frame Configuration
**Problem:** SLAM needs complete TF chain: `map → world → chassis_link → lidar`
**Solution:** Added static transform publishers in launch file since Isaac Sim's Transform Tree node doesn't publish

### 2. LiDAR Frame ID
**Problem:** Confusion about chassis_link vs lidar frame
**Solution:** Set Isaac Sim ROS2 RTX Lidar Helper frameId to `lidar`

### 3. SLAM Double-Activation Error
**Problem:** `Failed to make transition 'TRANSITION_CONFIGURE/ACTIVATE'`
**Cause:** SLAM Toolbox's online_async_launch.py auto-activates by default, causing conflict
**Solution:** Added `autostart: 'false'` to launch arguments

### 4. YAML Parameter Syntax
**Problem:** `ceres_loss_function: None` interpreted as null
**Solution:** Quote it: `ceres_loss_function: "None"`

### 5. Simulation Time Setup
**Problem:** Nodes not using sim_time
**Solution:**
- Isaac Sim: Unchecked "Use System Time" in RTX Lidar node
- Added ROS2 Publish Clock node in Action Graph
- Set `use_sim_time: true` in all parameter files

## Current Configuration

### TF Tree
```
map (from SLAM)
└── world (static, identity)
    └── chassis_link (static, identity)
        └── lidar (static, 0,0,0.5)
```

### Key Files
- Launch: `ros2_ws/src/isaac_nav_bringup/launch/isaac_slam_nav.launch.py`
- SLAM params: `ros2_ws/src/isaac_nav_bringup/config/slam_params.yaml`
- Nav2 params: `ros2_ws/src/isaac_nav_bringup/config/nav2_params.yaml`
- Isaac world: `isaac/worlds/simple_obstacles_carter_lidar.usd`

### SLAM Parameters
```yaml
use_sim_time: true
mode: mapping
odom_frame: world
map_frame: map
base_frame: chassis_link
scan_topic: /scan
resolution: 0.05
max_laser_range: 20.0
minimum_travel_distance: 0.2
minimum_travel_heading: 0.2
```

## Remaining Issue

**Symptom:** `/map` topic exists but never publishes data
**Verified:**
- SLAM node is `active [3]`
- Parameters loaded correctly
- Subscribed to `/scan` topic
- Scans are publishing at 3 Hz
- TF chain complete and valid
- Robot moving (cmd_vel commands working)

**Possible Causes:**
1. SLAM waiting for initial pose?
2. Coordinate frame mismatch in scan data?
3. Scan data format issue (Isaac Sim uses -1.0 for invalid, not inf)?
4. SLAM internal error not being logged?
5. Missing SLAM Toolbox dependency?

## Diagnostic Commands

```bash
# Check SLAM state
ros2 lifecycle get /slam_toolbox

# Check parameters
ros2 param get /slam_toolbox base_frame
ros2 param get /slam_toolbox odom_frame

# Check topics
ros2 topic hz /scan --use-sim-time
ros2 topic hz /map --use-sim-time
ros2 topic info /scan --verbose

# Check TF
ros2 run tf2_ros tf2_echo map lidar
ros2 run tf2_tools view_frames

# Test SLAM alone
ros2 run slam_toolbox async_slam_toolbox_node \
  --ros-args -p use_sim_time:=true \
  --params-file src/isaac_nav_bringup/config/slam_params.yaml
```

## Next Steps
1. Check SLAM Toolbox source code for map publishing logic
2. Try with default SLAM parameters from slam_toolbox package
3. Test with a simpler simulator (Gazebo) to isolate Isaac Sim issues
4. Add verbose logging to SLAM Toolbox
5. Check if SLAM needs explicit "start mapping" service call
