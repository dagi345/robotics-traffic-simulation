#!/bin/bash
# Verification Script for Traffic Control Fixes
# This script helps verify that all fixes are working correctly

echo "=========================================="
echo "Traffic Control System - Fix Verification"
echo "=========================================="
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to check if a topic exists
check_topic() {
    local topic=$1
    if ros2 topic list | grep -q "^${topic}$"; then
        echo -e "${GREEN}✓${NC} Topic exists: $topic"
        return 0
    else
        echo -e "${RED}✗${NC} Topic missing: $topic"
        return 1
    fi
}

# Function to check if a node is running
check_node() {
    local node=$1
    if ros2 node list | grep -q "$node"; then
        echo -e "${GREEN}✓${NC} Node running: $node"
        return 0
    else
        echo -e "${RED}✗${NC} Node not running: $node"
        return 1
    fi
}

echo "Step 1: Checking if simulation is running..."
echo "--------------------------------------------"

# Check if required nodes are running
check_node "traffic_flow_controller"
check_node "smart_traffic_manager"
check_node "intersection_zone_monitor"

echo ""
echo "Step 2: Checking traffic light topics..."
echo "--------------------------------------------"

# Check traffic light topics
check_topic "/traffic/light/north"
check_topic "/traffic/light/south"
check_topic "/traffic/light/east"
check_topic "/traffic/light/west"

echo ""
echo "Step 3: Checking vehicle command topics..."
echo "--------------------------------------------"

# Check a few vehicle topics (sample)
check_topic "/model/car_s1/cmd_vel"
check_topic "/model/car_n1/cmd_vel"
check_topic "/model/car_e1/cmd_vel"
check_topic "/model/car_w1/cmd_vel"

echo ""
echo "Step 4: Checking sensor topics..."
echo "--------------------------------------------"

check_topic "/sensors/north"
check_topic "/sensors/south"
check_topic "/sensors/east"
check_topic "/sensors/west"
check_topic "/sensors/intersection_zone"

echo ""
echo "Step 5: Sampling velocity commands..."
echo "--------------------------------------------"
echo "Checking if vehicles use correct velocity axes..."
echo ""

# Function to check velocity command
check_velocity() {
    local car=$1
    local expected_axis=$2
    local expected_sign=$3
    
    echo "Checking $car (expecting $expected_axis with $expected_sign sign)..."
    
    # Get one message from the topic (timeout after 2 seconds)
    local msg=$(timeout 2 ros2 topic echo --once /model/$car/cmd_vel 2>/dev/null)
    
    if [ -z "$msg" ]; then
        echo -e "${YELLOW}⚠${NC} No velocity data yet for $car"
        return 1
    fi
    
    # Extract linear.x and linear.y values
    local linear_x=$(echo "$msg" | grep -A 1 "linear:" | grep "x:" | awk '{print $2}')
    local linear_y=$(echo "$msg" | grep -A 1 "linear:" | grep "y:" | awk '{print $2}')
    
    # Check which axis is being used
    if [ "$expected_axis" = "x" ]; then
        if [ "$linear_x" != "0.0" ] && [ "$linear_y" = "0.0" ]; then
            # Check sign
            if [ "$expected_sign" = "positive" ]; then
                if (( $(echo "$linear_x > 0" | bc -l) )); then
                    echo -e "${GREEN}✓${NC} $car: Correct (linear.x > 0)"
                    return 0
                fi
            else
                if (( $(echo "$linear_x < 0" | bc -l) )); then
                    echo -e "${GREEN}✓${NC} $car: Correct (linear.x < 0)"
                    return 0
                fi
            fi
        fi
    else  # expected_axis = "y"
        if [ "$linear_y" != "0.0" ] && [ "$linear_x" = "0.0" ]; then
            # Check sign
            if [ "$expected_sign" = "positive" ]; then
                if (( $(echo "$linear_y > 0" | bc -l) )); then
                    echo -e "${GREEN}✓${NC} $car: Correct (linear.y > 0)"
                    return 0
                fi
            else
                if (( $(echo "$linear_y < 0" | bc -l) )); then
                    echo -e "${GREEN}✓${NC} $car: Correct (linear.y < 0)"
                    return 0
                fi
            fi
        fi
    fi
    
    echo -e "${RED}✗${NC} $car: Incorrect velocity axis/sign"
    echo "   linear.x: $linear_x, linear.y: $linear_y"
    return 1
}

# Check sample vehicles from each direction
check_velocity "car_s1" "y" "negative"
check_velocity "car_n1" "y" "positive"
check_velocity "car_e1" "x" "positive"
check_velocity "car_w1" "x" "negative"

echo ""
echo "Step 6: Checking traffic light states..."
echo "--------------------------------------------"
echo "Sampling current traffic light states..."
echo ""

# Sample traffic light states
for direction in north south east west; do
    echo -n "Light $direction: "
    state=$(timeout 1 ros2 topic echo --once /traffic/light/$direction 2>/dev/null | grep "data:" | awk '{print $2}' | tr -d "'")
    if [ -z "$state" ]; then
        echo -e "${YELLOW}⚠ No data${NC}"
    else
        if [ "$state" = "RED" ]; then
            echo -e "${RED}$state${NC}"
        elif [ "$state" = "YELLOW" ]; then
            echo -e "${YELLOW}$state${NC}"
        elif [ "$state" = "GREEN" ]; then
            echo -e "${GREEN}$state${NC}"
        else
            echo "$state"
        fi
    fi
done

echo ""
echo "=========================================="
echo "Verification Complete!"
echo "=========================================="
echo ""
echo "If all checks passed, the fixes are working correctly."
echo ""
echo "To monitor the system in real-time, use:"
echo "  ros2 topic echo /traffic/light/north"
echo "  ros2 topic echo /model/car_s1/cmd_vel"
echo ""
echo "To see vehicle positions:"
echo "  ros2 topic echo /sensors/north"
echo ""
