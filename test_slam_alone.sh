#!/bin/bash
cd ~/projects/Robotics/isaac_diff_drive_nav/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "=== Testing SLAM Toolbox with minimal params ==="
echo "This will show the actual error..."
echo ""

ros2 run slam_toolbox async_slam_toolbox_node \
  --ros-args \
  -p use_sim_time:=true \
  --params-file src/isaac_nav_bringup/config/slam_params.yaml

