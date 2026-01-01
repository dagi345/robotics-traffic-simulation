#!/bin/bash
# Debug script to check why cars aren't stopping at traffic lights

echo "=========================================="
echo "Traffic Light Debug - Checking System"
echo "=========================================="
echo ""

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo "Step 1: Checking if ROS2 nodes are running..."
echo "--------------------------------------------"
ros2 node list

echo ""
echo "Step 2: Checking traffic light topics..."
echo "--------------------------------------------"
echo "Checking /traffic/light/north..."
timeout 2 ros2 topic echo --once /traffic/light/north 2>/dev/null
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ North light is publishing${NC}"
else
    echo -e "${RED}✗ North light NOT publishing${NC}"
fi

echo ""
echo "Step 3: Checking if traffic_flow_controller is receiving lights..."
echo "--------------------------------------------"
ros2 topic info /traffic/light/north

echo ""
echo "Step 4: Checking vehicle velocity commands..."
echo "--------------------------------------------"
echo "Sampling car_s1 velocity..."
timeout 2 ros2 topic echo --once /model/car_s1/cmd_vel 2>/dev/null

echo ""
echo "Step 5: Checking sensor data..."
echo "--------------------------------------------"
echo "Checking if sensors detect vehicles..."
timeout 2 ros2 topic echo --once /sensors/north 2>/dev/null | head -20

echo ""
echo "=========================================="
echo "Quick Diagnosis Complete"
echo "=========================================="
echo ""
echo "If you see issues above, the problem is likely:"
echo "1. Nodes not running"
echo "2. Topics not publishing"
echo "3. Sensors not detecting vehicles"
echo ""
