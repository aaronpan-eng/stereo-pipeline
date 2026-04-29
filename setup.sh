#!/bin/bash
set -e

# pull submodules
# git submodule update --init --recursive

# update apt repo
sudo apt-get update

# pull lfs
# sudo apt-get install git-lfs
# git lfs install
# git lfs pull

# install pycuvslam stuff
cd submodules/pycuvslam/
pip install -e bin/aarch64
cd ../..

# install requirements from this folder
pip install -r requirements.txt
# can probably remove this
# python3 -m pip install --upgrade "packaging==24.2" "setuptools==79.0.1"

# install opencv and cv_bridge from source
# sudo apt install -y python3-colcon-common-extensions
cd scripts
printf 'yes\n' | ./install_opencv4.10.0_Jetpack6.1.sh
./install_cv_bridge.sh
cd ..

# install unzip tools
# sudo apt-get install unzip

# unzip pydbow3
cd external
if [ ! -d "PyDBoW3" ]; then
    sudo unzip PyDBoW3.zip
fi

# Build and install native DBoW3 dependency first.
cd PyDBoW3/modules/dbow3
cmake -S . -B build \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local
cmake --build build -j"$(nproc)"
sudo cmake --install build
export DBoW3_DIR="/usr/local/lib/cmake/DBoW3"

# install pydbow3
cd ../..
pip install -e .

# return to home directory
cd ../..

cd scripts
./install_v4l2_camera.sh

./install_torch.sh
