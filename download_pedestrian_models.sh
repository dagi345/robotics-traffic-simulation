#!/bin/bash

# Script to download pedestrian models for Gazebo simulation
# These models will be downloaded to the local Gazebo model directory

echo "================================================"
echo "Pedestrian Model Downloader for Gazebo Harmonic"
echo "================================================"
echo ""

# Set Gazebo model path
GAZEBO_MODEL_PATH="$HOME/.gz/models"
mkdir -p "$GAZEBO_MODEL_PATH"

echo "Models will be downloaded to: $GAZEBO_MODEL_PATH"
echo ""

# Function to download model from Gazebo Fuel
download_fuel_model() {
    local model_name=$1
    local owner=$2
    
    echo "Downloading $model_name from Gazebo Fuel..."
    
    # Use gz fuel download command
    gz fuel download -u "https://fuel.gazebosim.org/1.0/$owner/models/$model_name" -v 4
    
    if [ $? -eq 0 ]; then
        echo "✓ Successfully downloaded $model_name"
    else
        echo "✗ Failed to download $model_name"
    fi
    echo ""
}

# Download common pedestrian/human models
echo "Downloading pedestrian models..."
echo ""

# These are common actor models available in Gazebo
download_fuel_model "actor" "OpenRobotics"
download_fuel_model "person_walking" "OpenRobotics"
download_fuel_model "person_standing" "OpenRobotics"

# Additional human models
download_fuel_model "MaleVisitorPhone" "OpenRobotics"
download_fuel_model "MaleVisitorSit" "OpenRobotics"
download_fuel_model "ConstructionBarrier" "OpenRobotics"

echo "================================================"
echo "Download complete!"
echo ""
echo "Available models in: $GAZEBO_MODEL_PATH"
echo ""
echo "To use these models in your world file, reference them as:"
echo "  <uri>model://actor</uri>"
echo "  <uri>model://person_walking</uri>"
echo "  etc."
echo ""
echo "================================================"
