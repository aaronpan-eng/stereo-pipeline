# Stereo Pipeline

(WORK IN PROGRESS!!) ROS2-based stereo visual odometry and depth estimation pipline. Mainly for use in NeuFieldRobotics Lab.

## Overview
Todo
- [ ] Evaluate SLAM algos on drone data
  - [x] cuvslam traj
  - [x] droidslam traj
  - [x] orbslam3 traj (after parameter tuning - still doesnt track well)
  - [x] compare with all
  - [ ] troubleshoot trajectory differences
- [ ] Integrate Vimba stuff for AlliedVision cameras
- [ ] Check out suggestion for cuda accelerated rectification package (https://docs.nvidia.com/vpi/algo_ldc.html)
- [x] Create NeuStereo ROS2 node
- [ ] ** Convert nodes into cpp
- [x] Github action to check cuvslam-ros2 Dockerfile


Contains ros2 nodes for:
- **Stereo image rectification** from calibrated camera pairs
- **Visual odometry** using NVIDIA cuVSLAM (GPU-accelerated)
- **WIP - depth map output** with NeuStereo

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

## Docker Setup

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

### 1. Build the workspace (inside Docker)
```bash
cd /workspace
colcon build --symlink-install
source install/setup.bash
```

### 2. Launch stereo rectification
```bash
ros2 launch stereo_rectification rectify_launch.py config_yaml:=<your_config>.yaml
```

### 3. Launch cuVSLAM visual odometry
```bash
ros2 launch cuvslam_stereo cuvslam_stereo_launch.py
```

### 4. Play a ROS2 bag
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

