#!/bin/bash
set -e

# for jtop stuff
sudo pip3 install -U jetson-stats

# tensorrt runtime
sudo apt-get install -y \
    python3-libnvinfer-dev \
    python3-libnvinfer \
    tensorrt-libs \
    cuda-toolkit-12-6 \
    nvidia-tensorrt \
    libnvinfer10 \
    libnvinfer-plugin10 \
    nvidia-cudnn \
    nvidia-vpi \
    libnvonnxparsers10 \
    nvidia-l4t-dla-compiler \
    libnvinfer-bin \
    nvidia-tensorrt-dev 

# Fix the DLA compiler link (crucial for JetPack 6.x) 
sudo ln -sf /usr/lib/aarch64-linux-gnu/nvidia/libnvdla_compiler.so /usr/lib/aarch64-linux-gnu/tegra/libnvdla_compiler.so # Link Python bindings so jtop can "see" them sudo ln -sf /usr/lib/python3.10/dist-packages/tensorrt* /usr/local/lib/python3.10/dist-packages/ # Update the system library cache sudo ldconfig
