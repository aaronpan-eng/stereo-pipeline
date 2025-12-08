#!/bin/bash

# Check if container name argument is provided
if [ -z "$1" ]; then
    echo "Usage: ./run_docker.sh <container_name>"
    echo "Available containers:"
    docker ps -a --format "table {{.Names}}\t{{.Image}}\t{{.Status}}"
    exit 1
fi

CONTAINER_NAME=$1

xhost +local:docker

# Check if container exists
if [ "$(docker ps -aq -f name=^${CONTAINER_NAME}$)" ]; then
    echo "Container $CONTAINER_NAME already exists. Starting it..."
    docker start $CONTAINER_NAME
    docker exec -it $CONTAINER_NAME bash
else
    echo "Creating new container $CONTAINER_NAME..."
    docker run -it --gpus all --name $CONTAINER_NAME \
       --runtime=nvidia \
       --privileged \
       -e DISPLAY="$DISPLAY" \
       -e XAUTHORITY=$XAUTH \
       -e QT_X11_NO_MITSHM=1 \
       -e _X11_NO_MITSHM=1 \
       -e _MITSHM=0 \
       -v $XSOCK:$XSOCK \
       -v $XAUTH:$XAUTH \
       -e NVIDIA_DRIVER_CAPABILITIES=all \
       -v ../submodules:/workspace/submodules \
       -v ../src:/workspace/src \
       -v ../data:/workspace/data \
       -v /media/aaron/T5\ EVO1/payload1_20250828_1405:/workspace/data/payload1_20250828_1405 \
       -v /media/aaron/T5\ EVO1/2025-10-PRANCE:/workspace/data/2025-10-PRANCE \
       -v ../utils:/workspace/utils \
       -v /tmp/.X11-unix:/tmp/.X11-unix \
       droid-slam
fi
