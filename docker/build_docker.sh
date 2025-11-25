#!/bin/bash

echo "Rebuilding Docker image..."
docker build -t ros2-pycuvslam .

if [ $? -ne 0 ]; then
    echo "Build failed!"
    exit 1
fi
