#!/bin/bash
# Diagnostic script to check use_sim_time settings

echo "=== Checking /clock topic ==="
ros2 topic info /clock
echo ""

echo "=== Checking SLAM Toolbox use_sim_time ==="
ros2 param get /slam_toolbox use_sim_time 2>&1 || echo "SLAM Toolbox not running or param not set"
echo ""

echo "=== Checking Controller Server use_sim_time ==="
ros2 param get /controller_server use_sim_time 2>&1 || echo "Controller not running or param not set"
echo ""

echo "=== Checking all nodes ==="
ros2 node list
echo ""

echo "=== Checking /map topic ==="
ros2 topic info /map
echo ""

echo "=== Checking for slam_toolbox logs ==="
echo "Look in your launch terminal for lines containing 'slam_toolbox' or 'async_slam_toolbox'"
