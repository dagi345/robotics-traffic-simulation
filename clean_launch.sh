#!/bin/bash

echo "========================================="
echo "Clean Launch - Smart Traffic System"
echo "========================================="
echo ""

# Kill any existing processes
echo "1. Cleaning up old processes..."
pkill -9 -f "gz sim" 2>/dev/null
pkill -9 -f "ros_gz_bridge" 2>/dev/null
pkill -9 -f "traffic_flow" 2>/dev/null
pkill -9 -f "smart_traffic_manager" 2>/dev/null
pkill -9 -f "intersection_zone_monitor" 2>/dev/null
pkill -9 -f "ros2 launch" 2>/dev/null
sleep 2
echo "   ✓ Old processes cleaned"
echo ""

# Rebuild
echo "2. Rebuilding system..."
cd ros2_ws
colcon build --packages-select smart_traffic_system --symlink-install > /dev/null 2>&1
cd ..
echo "   ✓ Build complete"
echo ""

# Launch
echo "3. Launching simulation..."
echo "   - Gazebo will open in a new window"
echo "   - Traffic lights will start in NS_GREEN state"
echo "   - Vehicles will respond to traffic lights"
echo "   - Press Ctrl+C to stop"
echo ""

source ros2_ws/install/setup.bash
ros2 launch smart_traffic_system simulation.launch.py
