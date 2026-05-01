#!/bin/bash
set -e

sudo apt-get update
sudo apt-get install -y python3-pip libopenblas-dev

# set venv for torch install (to avoid conflicts with system packages)
python3 -m venv torchjp6
source torchjp6/bin/activate

python -m pip install -U pip wheel

python -m pip uninstall -y torch torchvision torchaudio || true

python -m pip install torch==2.8.0 torchvision==0.23.0 \
  --index-url=https://pypi.jetson-ai-lab.io/jp6/cu126
