#!/bin/bash
cd ros2_ws
source install/setup.bash
echo "Launching simulation for 10 seconds..."
timeout 10 ros2 launch smart_traffic_system simulation.launch.py 2>&1 | grep -E "ERROR|WARN|INFO.*Started|Activated" | head -20
echo ""
echo "If you saw 'Started' or 'Activated' messages, Gazebo is working!"
