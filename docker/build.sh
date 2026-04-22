#!/bin/bash
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
docker build -f "$REPO_ROOT/docker/Dockerfile_jetson" -t stereo-pipeline:latest "$REPO_ROOT"
