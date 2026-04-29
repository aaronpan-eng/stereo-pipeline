#!/bin/bash
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPO_NAME="$(basename "$REPO_ROOT")"
docker build \
  --build-arg "REPO_NAME=$REPO_NAME" \
  -f "$REPO_ROOT/docker/Dockerfile_jetson" \
  --network=host \
  -t stereo-pipeline:latest \
  "$REPO_ROOT"
