#!/bin/bash

echo "Rebuilding Docker image..."
docker build -f Dockerfile_orbslam3 -t orbslam3 .

if [ $? -ne 0 ]; then
    echo "Build failed!"
    exit 1
fi
