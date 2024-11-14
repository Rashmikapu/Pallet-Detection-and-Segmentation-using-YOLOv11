# Use the official ROS 2 Humble image as the base
FROM osrf/ros:humble-desktop-full

# Set environment variables to avoid warnings
ENV DEBIAN_FRONTEND=noninteractive

# Update apt and install necessary dependencies
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y \
    python3-pip \
    apt-utils \
    python3-colcon-common-extensions \
    python3-rosdep \
    build-essential \
    git \
    curl \
    wget \
    cmake \
    unzip \
    libboost-all-dev \
    libopencv-dev \
    libyaml-cpp-dev \
    libssl-dev \
    python3-setuptools \
    python3-dev \
    libeigen3-dev \
    libcurl4-openssl-dev \
    libpng-dev \
    libjpeg-dev \
    libtiff-dev \
    libopenblas-dev \
    liblapack-dev \
    terminator \
    nvidia-cuda-toolkit \
    nvidia-cuda-dev \
    && rm -rf /var/lib/apt/lists/*
RUN apt-get clean all -y

# Install apt-get installed sympy
RUN apt-get remove -y python3-sympy
RUN apt-get clean all -y

# Install PyTorch and other Python dependencies
RUN pip3 install --upgrade --force-reinstall pip
RUN pip3 install --upgrade --force-reinstall torch torchvision torchaudio

RUN pip3 install --upgrade --force-reinstall numpy
RUN rm -rf /var/lib/apt/lists/*
RUN apt-get clean all -y

# Install additional ROS dependencies
RUN apt-get update && apt-get install -y \
    ros-humble-ros-base \
    ros-humble-rclcpp \
    ros-humble-rclpy \
    ros-humble-std-msgs \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-rviz2 \
    ros-humble-navigation2 \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-turtlebot3* \
    ros-humble-turtlebot4-desktop \
    && rm -rf /var/lib/apt/lists/*
RUN apt-get clean all -y

# Install OpenCV dependencies (if needed)
RUN apt-get update && apt-get install -y \
    libopencv-dev \
    && rm -rf /var/lib/apt/lists/*
RUN pip install --upgrade numpy opencv-python

# Install the ultralytics package from PyPI
RUN pip install ultralytics
RUN rm -rf /var/lib/apt/lists/*
RUN apt-get clean all -y

# Source ROS 2 setup file and set entrypoint
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /ros_ws/install/setup.bash" >> ~/.bashrc

# Create a workspace directory
RUN mkdir -p /ros_ws/src

# Create an initial bash file to delete cv-bridge installation
RUN echo '#!/bin/bash' > /initial_setup.sh && \
		echo 'pip uninstall -y cv-bridge' >> /initial_setup.sh && \
		echo 'pip install cv_bridge' >> /initial_setup.sh

# Install rosdep and initialize it
RUN rosdep update

# Clean the packages after installations
RUN apt-get clean all -y

# Expose ROS 2 workspace port (optional)
EXPOSE 11311

# Set the default command
CMD ["/bin/bash"]

