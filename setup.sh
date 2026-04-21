#!/bin/bash
set -e

# pull submodules
git submodule update --init --recursive

# update apt repo
sudo apt-get update

# pull lfs
sudo apt-get install git-lfs
git lfs install
git lfs pull

# install pycuvslam stuff
cd submodules/pycuvslam/
pip install -e bin/aarch64
cd ../..

# install requirements from this folder
pip install -e .

# install opencv and cv_bridge from source
cd scripts
./install_opencv4.10.0_Jetpack6.1.sh
./install_cv_bridge.sh

# install unzip tools
sudo apt-get install unzip

# install pydbow3
cd external
if [ ! -d "PyDBoW3" ]; then
    sudo unzip PyDBoW3.zip
fi
cd PyDBoW3
pip install -e .

# return to home directory
cd ../..

