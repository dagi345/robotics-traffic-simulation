#!/bin/bash

echo "=== Testing Smart Traffic System Launch ==="
echo ""

# Source ROS 2
source /opt/ros/jazzy/setup.bash

# Source workspace
cd ros2_ws
source install/setup.bash

echo "1. Checking if world file exists..."
WORLD_FILE="install/smart_traffic_system/share/smart_traffic_system/worlds/intersection.world"
if [ -f "$WORLD_FILE" ]; then
    echo "   ✓ World file found: $WORLD_FILE"
else
    echo "   ✗ World file NOT found: $WORLD_FILE"
    exit 1
fi

echo ""
echo "2. Checking if bridge config exists..."
BRIDGE_CONFIG="install/smart_traffic_system/share/smart_traffic_system/config/bridge_config.yaml"
if [ -f "$BRIDGE_CONFIG" ]; then
    echo "   ✓ Bridge config found: $BRIDGE_CONFIG"
else
    echo "   ✗ Bridge config NOT found: $BRIDGE_CONFIG"
    exit 1
fi

echo ""
echo "3. Checking if traffic light model exists..."
MODEL_DIR="install/smart_traffic_system/share/smart_traffic_system/models/detailed_traffic_light"
if [ -d "$MODEL_DIR" ]; then
    echo "   ✓ Traffic light model found: $MODEL_DIR"
else
    echo "   ✗ Traffic light model NOT found: $MODEL_DIR"
    exit 1
fi

echo ""
echo "4. Checking if Python scripts are installed..."
for script in traffic_flow.py smart_traffic_manager.py intersection_zone_monitor.py; do
    SCRIPT_PATH="install/smart_traffic_system/lib/smart_traffic_system/$script"
    if [ -f "$SCRIPT_PATH" ]; then
        echo "   ✓ $script found"
    else
        echo "   ✗ $script NOT found"
    fi
done

echo ""
echo "5. Testing Gazebo launch (will run for 5 seconds)..."
echo "   Press Ctrl+C if Gazebo crashes immediately"
echo ""

timeout 5 ros2 launch smart_traffic_system simulation.launch.py 2>&1 | head -50

echo ""
echo "=== Test Complete ==="
