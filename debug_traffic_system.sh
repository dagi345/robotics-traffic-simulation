#!/bin/bash

echo "========================================="
echo "  TRAFFIC SYSTEM DEBUG SCRIPT"
echo "========================================="
echo ""

echo "1. Checking ROS nodes..."
ros2 node list | grep -E "(traffic|zone)"
echo ""

echo "2. Checking topic publishers/subscribers..."
echo "Traffic light state:"
ros2 topic info /traffic_lights/state
echo ""
echo "North light:"
ros2 topic info /traffic/light/north
echo ""

echo "3. Checking if messages are being published..."
echo "Waiting for traffic light state message..."
timeout 3 ros2 topic echo /traffic_lights/state --once
echo ""

echo "4. Checking individual light messages..."
echo "North light (waiting 3 seconds):"
timeout 3 ros2 topic echo /traffic/light/north --once
echo ""

echo "5. Checking vehicle positions from sensors..."
echo "North sensor (southbound vehicles):"
timeout 2 ros2 topic echo /sensors/north --once | head -20
echo ""

echo "6. Monitoring logs from traffic_flow_controller..."
echo "Looking for light reception and stop line detection..."
echo "(This will show the last 20 log messages)"
ros2 node list
echo ""

echo "========================================="
echo "  DEBUG COMPLETE"
echo "========================================="
echo ""
echo "If you see:"
echo "  ✅ Nodes running"
echo "  ✅ Topics have publishers/subscribers"
echo "  ✅ Messages being published"
echo "  ✅ Vehicle positions being tracked"
echo ""
echo "Then the system should be working."
echo ""
echo "If individual lights show no messages, check:"
echo "  - Is update_lights() being called?"
echo "  - Is _publish_individual_light_states() being called?"
echo "  - Are the publishers created correctly?"
