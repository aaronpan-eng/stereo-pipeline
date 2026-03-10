# Stereo Pipeline

![Experimental](https://img.shields.io/badge/experimental-work%20in%20progress-red)

(WORK IN PROGRESS!!) ROS2-based stereo visual odometry and depth estimation pipline. Mainly for use in NeuFieldRobotics Lab.

## Overview
### Todo:

*In **Bold** are high priority tasks*

- [ ] **(WIP) - Get AlliedVision cameras working with the pipeline**
- [ ] **(WIP) - Debug why CUDA is running out of memory**
- [ ] **(WIP) - Evaluate runtimes of image publishing, rectification node, cuvslam node, neustereo node**
- [ ] Add Shell script or instructions for building OpenCV with CUDA support and cv_bridge
- [ ] Add ros2 camera launch command to the full_pipeline_launch.py
- [ ] After optimization - Convert necessary nodes into cpp
- [ ] Add IMU for VIO and compare traj once dslam comparison sorted
- [x] Add GPU suport for OpenCV (built OpenCV from source and used OpenCV CUDA module)
  - [x] ~~(WIP) - Check out suggestion for cuda accelerated rectification package (https://docs.nvidia.com/vpi/algo_ldc.html)~~
- [ ] Evaluate SLAM algos on drone data
  - [x] cuvslam traj
  - [x] droidslam traj
  - [x] orbslam3 traj (after parameter tuning - still doesnt track well)
  - [x] compare with all
  - [ ] troubleshoot trajectory differences
- [x] Create NeuStereo ROS2 node
  - [x] Get NeuStereo ROS2 disparity output working
- [ ] Github action to check cuvslam-ros2 Dockerfile
  - [ ] Corrrect Github action to check cuvslam-ros2 Dockerfile
  - [x] Initial Github action to check Dockerfile


Contains ros2 nodes for:
- **Stereo image rectification** from calibrated camera pairs
- **Visual odometry** using NVIDIA cuVSLAM (GPU-accelerated)
- **Depth map output** with NeuStereo

Evaluation of NeuRoam data with fast_LIMO
- **LiDAR-inertial odometry** using fast_LIMO in `sumbodules/fast_LIMO`

Tools for
- **Trajectory evaluation and visualization** using the [evo package](https://github.com/MichaelGrupp/evo)

## Repository Structure

```
stereo-pipeline/
├── src/                        # ROS2 packages
│   ├── stereo_rectification/   # Stereo image rectification node
│   ├── cuvslam_stereo/         # cuVSLAM visual odometry node
│   └── launcher/               # WIP combined launch files
├── submodules/                 
│   ├── pycuvslam/              # NVIDIA cuVSLAM
│   ├── fast_LIMO/              # LiDAR-inertial odometry
│   ├── ORB_SLAM3/              # ORB-SLAM3 (optional)
│   ├── droid-slam/             # DROID-SLAM (optional)
│   └── NeuROAM/                # Sensor drivers
├── docker/                     # Docker build and run scripts
├── utils/                      
│   └── ...                     # Plotting and calibration utils
└── data/                       # Dataset storage
```

## Prerequisites (WIP)

### Pull submodules
```
git submodule update --init --recursive
```

### Pull weights and large files using git lfs
Make sure git lfs is installed, then pull
```
sudo apt-get install git-lfs
# in the repo directory:
git lfs install
git lfs pull
```

### Install requirements.txt
```
# inside /stereo-pipeline directory
pip install -r requirements.txt
```

### Install OpenCV with CUDA support and build and source cv_bridge
```
# inside /scripts
./install_opencv4.10.0_Jetpack6.1.sh
./install_cv_bridge.sh
```

### Reboot machine
```
sudo reboot
```

## Docker Setup *(skip if running on Jetson)*

### Build the Docker image for cuvslam
```bash
cd docker
./build_docker.sh
```

### Run the container
```bash
./run_docker.sh <container_name>
```

This starts a container with:
- ROS2 Humble
- NVIDIA GPU support
- cuVSLAM installed
- Zenoh middleware for ROS2 bag playback

## ROS2 Quick Start

### 1. Build the workspace inside stereo-pipeline directory
```bash
colcon build --symlink-install
source install/setup.bash
```

### 2. Launch full pipeline
```bash
ros2 launch launcher full_pipeline_launch.py
```


> #### Launch components separately
>
> ##### Launch stereo rectification only
> ```bash
> ros2 launch stereo_rectification rectify_launch.py config_yaml:=<your_config>.yaml
> ```
>
> ##### Launch cuVSLAM visual odometry only
> ```bash
> ros2 launch cuvslam_stereo cuvslam_stereo_launch.py rerun_visualization:=<true/false> save_trajectory_tum:=<true/false>
> ```
> 
> #### Launch NeuStereo depth estimation only
> ```bash
> ros2 launch neustereo_ros2 neustereo_launch.py model_config_yaml:=<your_model_config>.yaml
> ```


### 3. Play a ROS2 bag
```bash
ros2 bag play <path_to_bag>
```

## Configuration

Camera calibration files are stored in `src/stereo_rectification/config/` in YAML format:

```yaml
rectify_stereo_imgs:
  ros__parameters:
    cam0:
      intrinsics: [fx, fy, cx, cy]
      distortion_coeffs: [k1, k2, p1, p2, k3]
      resolution: [width, height]
    cam1:
      T_cn_cnm1: [4x4 transformation matrix flattened]
      intrinsics: [fx, fy, cx, cy]
      distortion_coeffs: [k1, k2, p1, p2, k3]
```

## Utilities

### Plot trajectories
```bash
python utils/plot_results.py -i traj1.txt traj2.txt -o ./output -n 2
```

### Evaluating trajectories

Refer to the [evo](https://github.com/MichaelGrupp/evo) package for evaluating trajectories.

## Output Format

Trajectories are saved in TUM format:
```
timestamp tx ty tz qx qy qz qw
```

## Dependencies

- ROS2 Humble
- NVIDIA GPU with CUDA
- OpenCV
- NumPy, Pandas, Matplotlib
- Rerun (visualization)
- ADD OTHER STUFF HERE 

