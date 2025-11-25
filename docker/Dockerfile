FROM pycuvslam:realsense-x86

# Set environment variables for non-interactive installation
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=America/New_York

# Set locale (required by ROS2)
RUN apt-get update && apt-get install -y locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8

# Install ROS2 prerequisites (curl already exists in base image based on Dockerfile)
RUN apt-get update && apt-get install -y \
    software-properties-common \
    && add-apt-repository universe \
    && rm -rf /var/lib/apt/lists/*

# Add ROS2 GPG key and repository
RUN apt-get update && apt-get install -y curl \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 Humble and required packages
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-humble-cv-bridge \
    ros-humble-rosbag2-storage-mcap \
    ros-humble-stereo-image-proc \
    ros-humble-image-transport \
    ros-humble-image-pipeline \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*


# Install fast LIMO prerequisites
RUN apt-get update && apt-get install -y \
    net-tools \
    git \
    wget \
    inetutils-ping \
    x11-apps \
    vim

# Install Python packages (pip3 already upgraded in base image)
RUN pip3 install \
    "numpy<2" \
    opencv-python \
    pandas

# Initialize rosdep
RUN rosdep init || true && rosdep update

# Create workspace directory
WORKDIR /workspace

# Preserve PYTHONPATH
ENV PYTHONPATH=/opt/librealsense/build/Release:$PYTHONPATH

# Source ROS2 setup in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]
