#!/bin/bash

echo "========================================="
echo "  RESTARTING WITH TRAFFIC LIGHT FIX"
echo "========================================="
echo ""

echo "Step 1: Rebuilding package..."
cd ~/robotics-project/ros2_ws
colcon build --packages-select smart_traffic_system
echo "✅ Build complete"
echo ""

echo "Step 2: Sourcing workspace..."
source install/setup.bash
echo "✅ Workspace sourced"
echo ""

echo "Step 3: Launching simulation..."
echo "⚠️  Watch for vehicles stopping at stop lines!"
echo "⚠️  Traffic lights should cycle: GREEN → YELLOW → RED"
echo ""
echo "Press Ctrl+C to stop the simulation"
echo ""
sleep 2

ros2 launch smart_traffic_system simulation.launch.py
