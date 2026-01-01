#!/bin/bash

# Test script to verify traffic light system is working

echo "=== Testing Traffic Light System ==="
echo ""

echo "1. Checking if nodes are running..."
ros2 node list | grep -E "(traffic|zone)"
echo ""

echo "2. Checking traffic light state..."
timeout 2 ros2 topic echo /traffic_lights/state --once
echo ""

echo "3. Checking individual light states..."
echo "North light:"
timeout 2 ros2 topic echo /traffic/light/north --once
echo ""
echo "South light:"
timeout 2 ros2 topic echo /traffic/light/south --once
echo ""
echo "East light:"
timeout 2 ros2 topic echo /traffic/light/east --once
echo ""
echo "West light:"
timeout 2 ros2 topic echo /traffic/light/west --once
echo ""

echo "4. Monitoring state transitions (10 seconds)..."
timeout 10 ros2 topic echo /traffic_lights/state
