FROM nvidia/cuda:12.9.0-devel-ubuntu24.04 AS cuda

# ---- Final stage ----
FROM osrf/ros:jazzy-desktop-full

# Copy CUDA toolchain from the CUDA stage
COPY --from=cuda /usr/local/cuda /usr/local/cuda

# CUDA environment
ENV PATH=/usr/local/cuda/bin:${PATH} \
    LD_LIBRARY_PATH=/usr/local/cuda/lib64:${LD_LIBRARY_PATH}

# Set environment for CUDA 12.9
ENV CUDA_HOME=/usr/local/cuda-12.9
ENV PATH=$CUDA_HOME/bin:$PATH
ENV LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility
ENV NVIDIA_REQUIRE_CUDA="cuda>=12.9"


# Dev user mapped to host UID/GID
ARG USERNAME=robat
ARG USER_UID=1000
ARG USER_GID=1000
RUN set -e; \
    getent group "${USER_GID}" >/dev/null || groupadd -g "${USER_GID}" "${USERNAME}"; \
    if ! getent passwd "${USER_UID}" >/dev/null; then \
        useradd -m -s /bin/bash -u "${USER_UID}" -g "${USER_GID}" -G sudo "${USERNAME}"; \
    else \
        usermod -l "${USERNAME}" "$(getent passwd ${USER_UID} | cut -d: -f1)" || true; \
        usermod -d "/home/${USERNAME}" -m "${USERNAME}" || true; \
        usermod -aG sudo "${USERNAME}" || true; \
    fi && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >/etc/sudoers.d/90-${USERNAME}

# Workspace skeleton + QoL
WORKDIR /ros2_ws
RUN mkdir -p /ros2_ws/src && chown -R ${USER_UID}:${USER_GID} /ros2_ws
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /etc/skel/.bashrc && \
    echo 'source /ros2_ws/install/setup.bash 2>/dev/null || true' >> /etc/skel/.bashrc && \
    cp /etc/skel/.bashrc /root/.bashrc

USER root

# Update packages
RUN sudo apt update && sudo apt upgrade -y

# Base tools + ROS 2 repo
RUN apt-get update && apt-get install -y --no-install-recommends \
      locales \
      tzdata \
      ca-certificates \
      curl \
      gnupg2 \
      lsb-release \
      software-properties-common \
      build-essential \
      git \
      wget \
      vim \
      htop \
      openssh-client sudo \
      python3-pip \
      python3-venv \
      python3-rosdep \
      python3-colcon-common-extensions \
      build-essential \
      cmake \
      bzip2 \
      g++ \
      zip \
      unzip \
    && locale-gen en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# ROS + your stack
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-desktop \
      ros-jazzy-moveit \
      ros-jazzy-moveit-visual-tools \
      ros-jazzy-ros2-control \
      ros-jazzy-ros2-controllers \
      ros-jazzy-controller-manager \
      ros-jazzy-joint-state-broadcaster \
      ros-jazzy-joint-trajectory-controller \
      ros-jazzy-rviz2 \
      ros-jazzy-xacro \
      ros-jazzy-ament-cmake \
      ros-jazzy-ament-cmake-python \
      ros-jazzy-robot-state-publisher \
      ros-jazzy-rqt-tf-tree \
      ros-jazzy-tf2-tools \
      ros-jazzy-tf-transformations \
      ros-jazzy-pcl-ros \
    && rm -rf /var/lib/apt/lists/*

# --- venv used by colcon & your nodes ---
RUN python3 -m venv /opt/venv
ENV PATH=/opt/venv/bin:$PATH \
    COLCON_PYTHON_EXECUTABLE=/opt/venv/bin/python3

USER ${USERNAME}

# GPU / DDS env
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    ROS_DOMAIN_ID=0 \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=all

# Known hosts for GitHub (ssh-agent convenience)
RUN mkdir -p ~/.ssh && ssh-keyscan -t rsa github.com >> ~/.ssh/known_hosts 2>/dev/null || true

CMD ["bash"]