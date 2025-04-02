# Use NVIDIA's official CUDA 12.6.1 base image with Ubuntu 24.04
FROM nvidia/cuda:12.6.1-base-ubuntu24.04

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies for adding repositories
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    ca-certificates \
    software-properties-common

# Add ROS 2 repository and key
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" | tee /etc/apt/sources.list.d/ros2.list

# Install ROS 2 Jazzy (Desktop Full)
RUN apt-get update && apt-get install -y ros-jazzy-desktop-full

# # Ensure NVIDIA libraries are correctly linked
# ENV NVIDIA_VISIBLE_DEVICES all
# ENV NVIDIA_DRIVER_CAPABILITIES compute,utility
# ENV NVIDIA_REQUIRE_CUDA "cuda>=12.2"

# # Set up CUDA paths
# ENV PATH /usr/local/cuda/bin:${PATH}
# ENV LD_LIBRARY_PATH /usr/local/cuda/lib64:${LD_LIBRARY_PATH}

# Add non-root user with same UID and GID as the host system
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN if ! id -u $USER_UID >/dev/null 2>&1; then \
        groupadd --gid $USER_GID $USERNAME && \
        useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi

# Add sudo support for the non-root user
RUN apt-get update && \
    apt-get install -y sudo && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Create workspace directory and set permissions
RUN sudo mkdir -p /ros2_ws/src && \
    sudo chown -R $USERNAME:$USERNAME /ros2_ws

# Switch to non-root user
USER $USERNAME

# Set workspace as working directory
WORKDIR /ros2_ws/src

# Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video $USERNAME

# Update all packages
RUN sudo apt update && sudo apt upgrade -y

# Install required system dependencies
RUN sudo apt install -y --no-install-recommends \
    curl \
    wget \
    gnupg2 \
    ca-certificates \
    lsb-release \
    software-properties-common \
    vim \
    htop \
    git \
    python3-pip \
    python3-tk \
    libpcl-dev \
    mesa-utils \
    libglx-mesa0 \
    libglu1-mesa \
    libglfw3-dev

# Install ROS 2 Jazzy dependencies (excluding unavailable packages)
RUN sudo apt install -y --no-install-recommends \
    ros-jazzy-ros-base \
    ros-jazzy-pcl-ros \
    ros-jazzy-rviz2 \
    ros-jazzy-gazebo-msgs \
    ros-jazzy-moveit \
    ros-jazzy-moveit-core \
    ros-jazzy-moveit-ros-planning \
    ros-jazzy-moveit-ros-move-group \
    ros-jazzy-moveit-ros-planning-interface \
    ros-jazzy-moveit-visual-tools \
    ros-jazzy-controller-manager \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-effort-controllers \
    ros-jazzy-position-controllers \
    ros-jazzy-joint-trajectory-controller \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-rqt \
    ros-jazzy-rqt-common-plugins \
    ros-jazzy-xacro \
    ros-jazzy-tf-transformations

# Clone missing Gazebo packages as non-root user
# RUN git clone -b jazzy https://github.com/ros-simulation/gazebo_ros_pkgs.git && \
#     git clone -b jazzy https://github.com/ros-controls/gazebo_ros2_control.git

# Set workspace as working directory for building
WORKDIR /ros2_ws

# Build ROS 2 workspace with missing Gazebo packages
# RUN colcon build --symlink-install

# Install Python virtual environment tools
RUN sudo apt install -y python3-venv

# Create a virtual environment in a user-writable directory
RUN python3 -m venv /home/$USERNAME/venv

# Switch to non-root user
USER $USERNAME
ENV PATH="/home/$USERNAME/venv/bin:$PATH"

# Install Python dependencies inside the virtual environment
RUN pip install --no-cache-dir --upgrade \
    numpy \
    pandas \
    scikit-learn \
    GPy \
    matplotlib \
    torch \
    transformers \
    protobuf

# Install colcon (ROS 2 build system)
RUN sudo apt install -y python3-colcon-common-extensions

# Ensure SSH authentication works inside the container
ENV SSH_AUTH_SOCK=/ssh-agent
RUN mkdir -p ~/.ssh && ssh-keyscan -t rsa github.com >> ~/.ssh/known_hosts

# Rosdep update and install dependencies
RUN sudo rosdep update && \
    rosdep install --from-paths /ros2_ws/src -i -y --rosdistro jazzy || echo "Skipping rosdep, repo not mounted yet"

# Update ROS entrypoint script to source workspace
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Set Bash as the default shell
SHELL ["/bin/bash", "-c"]

# Keep the container running
CMD ["bash"]