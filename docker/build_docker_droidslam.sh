#!/bin/bash

echo "Rebuilding Docker image..."
docker build -f Dockerfile_droidslam -t droid-slam .

if [ $? -ne 0 ]; then
    echo "Build failed!"
    exit 1
fi
