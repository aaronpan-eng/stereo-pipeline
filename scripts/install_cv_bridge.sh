#!/bin/bash

# Script to clone, build, and source vision_opencv (cv_bridge) for ROS2 Humble

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"
SUBMODULES_DIR="$WORKSPACE_DIR/submodules"
VISION_OPENCV_DIR="$SUBMODULES_DIR/vision_opencv"

echo "Installing cv_bridge from vision_opencv"

# Clone if it doesn't exist
if [ ! -d "$VISION_OPENCV_DIR" ]; then
    echo "** Cloning vision_opencv (humble branch)"
    cd "$WORKSPACE_DIR"
    git submodule add -b humble https://github.com/ros-perception/vision_opencv.git submodules/vision_opencv
else
    echo "** vision_opencv already exists at $VISION_OPENCV_DIR, skipping clone"
fi

# Build vision_opencv
echo "Building vision_opencv..."

cd "$VISION_OPENCV_DIR"

# Source ROS2 environment if not already sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "** Sourcing ROS2 Humble..."
    source /opt/ros/humble/setup.bash
fi

# Build using colcon
colcon build --symlink-install

# Source the built workspace
echo "Sourcing vision_opencv install..."
source "$VISION_OPENCV_DIR/install/setup.bash"
echo "cv_bridge installation complete!"
