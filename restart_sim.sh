#!/bin/bash

echo "Restarting simulation with fixed car orientations..."

# Kill old processes
pkill -9 -f "gz sim" 2>/dev/null
pkill -9 -f "ros_gz_bridge" 2>/dev/null
pkill -9 -f "traffic" 2>/dev/null
sleep 2

# Rebuild
cd ros2_ws
colcon build --packages-select smart_traffic_system --symlink-install > /dev/null 2>&1
cd ..

# Launch
source ros2_ws/install/setup.bash
ros2 launch smart_traffic_system simulation.launch.py
