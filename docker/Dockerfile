FROM nvidia/cuda:12.9.0-devel-ubuntu24.04

ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Etc/UTC \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8

SHELL ["/bin/bash", "-ceuo", "pipefail"]

# Base OS deps, locales, and common tools
RUN apt-get update && apt-get install -y --no-install-recommends \
      locales tzdata ca-certificates curl gnupg2 lsb-release \
      software-properties-common \
      build-essential git wget vim htop \
      openssh-client \
    && locale-gen en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# Add ROS 2 repository and key (Jazzy -> Ubuntu 24.04 "noble")
RUN mkdir -p /etc/apt/keyrings \
 && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" \
    > /etc/apt/sources.list.d/ros2.list

# Install ROS 2 Jazzy + dev tools
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-desktop \
      python3-rosdep python3-colcon-common-extensions \
      python3-pip python3-venv \
    && rosdep init || true \
    && rm -rf /var/lib/apt/lists/*

# MoveIt + common robotics packages
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-moveit \
      ros-jazzy-moveit-visual-tools \
      ros-jazzy-ros2-control \
      ros-jazzy-ros2-controllers \
      ros-jazzy-controller-manager \
      ros-jazzy-joint-state-broadcaster \
      ros-jazzy-joint-trajectory-controller \
      ros-jazzy-rviz2 \
      ros-jazzy-xacro \
      ros-jazzy-robot-state-publisher \
    && rm -rf /var/lib/apt/lists/*

# Optional GPU/OpenGL + PCL libs
RUN apt-get update && apt-get install -y --no-install-recommends \
      libpcl-dev \
      mesa-utils libglx-mesa0 libglu1-mesa libglfw3-dev \
    && rm -rf /var/lib/apt/lists/*

# --- safe non-root user (reuses existing UID/GID if present) ---
ARG USERNAME=robat
ARG USER_UID=1000
ARG USER_GID=${USER_UID}
RUN set -euo pipefail; \
    # Ensure group with USER_GID exists
    getent group "${USER_GID}" >/dev/null || groupadd -g "${USER_GID}" "${USERNAME}"; \
    # Ensure user with USER_UID exists
    if ! getent passwd "${USER_UID}" >/dev/null; then \
        useradd -m -s /bin/bash -u "${USER_UID}" -g "${USER_GID}" "${USERNAME}"; \
    fi; \
    # Resolve canonical username owning USER_UID
    UNAME="$(getent passwd "${USER_UID}" | cut -d: -f1)"; \
    apt-get update && apt-get install -y --no-install-recommends sudo && \
    echo "${UNAME} ALL=(root) NOPASSWD:ALL" > "/etc/sudoers.d/${UNAME}" && chmod 0440 "/etc/sudoers.d/${UNAME}" && \
    usermod -aG video "${UNAME}" || true && \
    mkdir -p /ros2_ws/src && chown -R "${USER_UID}:${USER_GID}" /ros2_ws && \
    rm -rf /var/lib/apt/lists/*

# Auto-source ROS + workspace for all shells
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /etc/bash.bashrc \
 && echo '[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash || true' >> /etc/bash.bashrc

# ---- venv: fixed path; create as root, chown to user; then switch user ----
RUN python3 -m venv /opt/venv && chown -R ${USER_UID}:${USER_GID} /opt/venv
ENV VIRTUAL_ENV=/opt/venv
ENV PATH="/opt/venv/bin:${PATH}"

# Switch to non-root using numeric IDs (works regardless of username)
USER ${USER_UID}:${USER_GID}
WORKDIR /ros2_ws

# Pip deps (installed into /opt/venv)
RUN pip install --no-cache-dir --upgrade pip \
 && pip install --no-cache-dir \
      numpy pandas scikit-learn GPy matplotlib transformers protobuf \
      torch

# SSH known_hosts for GitHub
RUN mkdir -p ~/.ssh \
 && ssh-keyscan -t rsa github.com >> ~/.ssh/known_hosts 2>/dev/null || true

# Rosdep update placeholder (OK if src empty)
RUN rosdep update || true \
 && rosdep install --from-paths /ros2_ws/src -i -y --rosdistro jazzy || true

SHELL ["/bin/bash", "-l", "-c"]
CMD ["bash"]
