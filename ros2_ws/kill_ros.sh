#!/bin/bash
# Kill ROS2 nodes
pkill -9 -f "nav2_"
pkill -9 -f "slam_toolbox"
pkill -9 -f "robot_state_publisher"
pkill -9 -f "ros2 launch"
pkill -9 -f "rviz2"

# Kill Gazebo (both server AND client)
pkill -9 -f "gz sim"
pkill -9 -f "gz "

echo "Waiting for DDS + Gazebo to fully stop..."
sleep 5
echo "Done. Safe to relaunch."
