#!/bin/bash

# Exit on error
set -e

echo "Starting Smart Traffic Light System Workspace Setup..."

# 1. Update and install basic dependencies
sudo apt update
sudo apt install -y ros-jazzy-ros-gz ros-jazzy-vision-msgs python3-colcon-common-extensions

# 2. Source ROS 2
source /opt/ros/jazzy/setup.bash

# 3. Create workspace if not exists (already done by agent, but for completeness)
WORKSPACE_DIR="/home/dagi/robotics-project/ros2_ws"
cd $WORKSPACE_DIR

# 4. Install dependencies using rosdep
# Note: This might require sudo and user input, but we'll try to skip-keys for now
# rosdep update
# rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy

# 5. Build the workspace
echo "Building the workspace..."
colcon build --symlink-install

# 6. Instructions for user
echo "----------------------------------------------------"
echo "Setup complete!"
echo "To use the workspace, run:"
echo "source $WORKSPACE_DIR/install/setup.bash"
echo "To launch the initial world (once created):"
echo "ros2 launch smart_traffic_system simulation.launch.py"
echo "----------------------------------------------------"
