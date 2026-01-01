#!/bin/bash

echo "================================================"
echo "Testing Pedestrian Integration"
echo "================================================"
echo ""

# Check if model is installed
echo "1. Checking if DoctorFemaleWalk model is installed..."
if [ -d "$HOME/.gz/models/DoctorFemaleWalk" ]; then
    echo "   ✅ Model found at ~/.gz/models/DoctorFemaleWalk"
else
    echo "   ❌ Model NOT found!"
    echo "   Run: cp -r ros2_ws/src/gazebo-ros-actor-plugin/config/skins/DoctorFemaleWalk ~/.gz/models/"
    exit 1
fi
echo ""

# Check if plugin is built
echo "2. Checking if gazebo_ros_actor_plugin is built..."
if [ -f "ros2_ws/install/gazebo_ros_actor_plugin/lib/libgazebo_ros_actor_plugin.so" ]; then
    echo "   ✅ Plugin built successfully"
else
    echo "   ❌ Plugin NOT built!"
    echo "   Run: cd ros2_ws && colcon build --packages-select gazebo_ros_actor_plugin"
    exit 1
fi
echo ""

# Check if world file has pedestrians
echo "3. Checking if intersection.world has pedestrians..."
if grep -q "pedestrian_north_1" ros2_ws/src/smart_traffic_system/worlds/intersection.world; then
    echo "   ✅ Pedestrians found in world file"
    
    # Count pedestrians
    ped_count=$(grep -c "name>pedestrian_" ros2_ws/src/smart_traffic_system/worlds/intersection.world)
    echo "   ✅ Total pedestrians: $ped_count"
else
    echo "   ❌ Pedestrians NOT found in world file!"
    exit 1
fi
echo ""

# Check if smart_traffic_system is built
echo "4. Checking if smart_traffic_system is built..."
if [ -d "ros2_ws/install/smart_traffic_system" ]; then
    echo "   ✅ Package built successfully"
else
    echo "   ❌ Package NOT built!"
    echo "   Run: cd ros2_ws && colcon build --packages-select smart_traffic_system"
    exit 1
fi
echo ""

echo "================================================"
echo "✅ All checks passed!"
echo "================================================"
echo ""
echo "To launch the simulation with pedestrians:"
echo ""
echo "  cd ros2_ws"
echo "  source install/setup.bash"
echo "  ros2 launch smart_traffic_system simulation.launch.py"
echo ""
echo "You should see:"
echo "  - 16 vehicles at the intersection"
echo "  - 8 pedestrians at crosswalks (2 per crosswalk)"
echo "  - Pedestrians standing in waiting positions"
echo ""
echo "Next steps:"
echo "  1. Launch simulation to verify pedestrians appear"
echo "  2. Implement pedestrian sensors"
echo "  3. Create traffic_controller_node.py"
echo "  4. Create pedestrian_controller.py"
echo ""
echo "See IMPLEMENTATION_ROADMAP.md for details"
echo "================================================"
