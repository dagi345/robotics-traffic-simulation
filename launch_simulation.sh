#!/bin/bash

echo "========================================="
echo "  Smart Traffic System Simulation"
echo "========================================="
echo ""

cd ros2_ws
source install/setup.bash

echo "Starting simulation..."
echo "- Gazebo will open in a new window"
echo "- Traffic lights will start in NS_GREEN state"
echo "- Vehicles will respond to traffic lights"
echo "- Yellow lights are now active (3 second duration)"
echo "- Intersection zone monitoring prevents collisions"
echo ""
echo "Press Ctrl+C to stop the simulation"
echo ""

ros2 launch smart_traffic_system simulation.launch.py
