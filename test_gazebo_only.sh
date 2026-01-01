#!/bin/bash
# Test Gazebo loading with proper environment

echo "=== Testing Gazebo World Loading ==="
echo "Setting up environment..."

# Source ROS 2
source ros2_ws/install/setup.bash

# Set Gazebo model path
export GZ_SIM_RESOURCE_PATH="$PWD/ros2_ws/install/smart_traffic_system/share/smart_traffic_system/models"

echo "Model path: $GZ_SIM_RESOURCE_PATH"
echo ""
echo "Launching Gazebo (will run for 10 seconds)..."
echo "If Gazebo window opens and stays open, the fix worked!"
echo ""

# Launch Gazebo with timeout
timeout 10 gz sim ros2_ws/install/smart_traffic_system/share/smart_traffic_system/worlds/intersection.world

echo ""
echo "=== Test Complete ==="
echo "If Gazebo opened and you saw the intersection with cars, the fix is successful!"
