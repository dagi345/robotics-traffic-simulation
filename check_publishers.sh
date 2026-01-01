#!/bin/bash

echo "Checking if traffic light publishers exist..."
echo ""

echo "=== Traffic Light State Topic ==="
ros2 topic info /traffic_lights/state
echo ""

echo "=== Individual Light Topics ==="
echo "North:"
ros2 topic info /traffic/light/north
echo ""

echo "South:"
ros2 topic info /traffic/light/south
echo ""

echo "East:"
ros2 topic info /traffic/light/east
echo ""

echo "West:"
ros2 topic info /traffic/light/west
echo ""

echo "=== Summary ==="
echo "Each topic should show:"
echo "  Publisher count: 1 (smart_traffic_manager)"
echo "  Subscription count: 1 (traffic_flow_controller)"
echo ""

echo "If Publisher count is 0, the manager is not creating publishers correctly."
echo "If Subscription count is 0, the controller is not subscribing correctly."
