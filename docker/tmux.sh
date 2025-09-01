#!/usr/bin/env bash
set -euo pipefail

# --- config (change if your container names differ) ---
ISAAC_CTR="${ISAAC_CTR:-isaac-lab-ros2}"
MOVEIT_CTR="${MOVEIT_CTR:-moveit_jazzy_cuda_container}"
SESSION_NAME="${SESSION_NAME:-robot}"

# --- sanity checks ---
command -v tmux >/dev/null 2>&1 || { echo "tmux not found. Install tmux first."; exit 1; }
docker ps --format '{{.Names}}' | grep -qx "$ISAAC_CTR"  || { echo "Container not running: $ISAAC_CTR"; exit 1; }
docker ps --format '{{.Names}}' | grep -qx "$MOVEIT_CTR"  || { echo "Container not running: $MOVEIT_CTR"; exit 1; }

# Helper to send a command to a pane (pane target like: "robot:isaaclab.0")
send() {
  # send text literally (-l) so quotes/semicolons aren't re-parsed by your shell or tmux
  tmux send-keys -t "$1" -l -- "$2"
  tmux send-keys -t "$1" C-m
}

# attach if session already exists
if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
  exec tmux attach -t "$SESSION_NAME"
fi

# Create session with first window
tmux new-session -d -s "$SESSION_NAME" -n isaaclab

# Build 2x2 layout in Window 1 (isaaclab)
tmux split-window -h -t "${SESSION_NAME}:isaaclab"
tmux select-pane -t "${SESSION_NAME}:isaaclab".0
tmux split-window -v -t "${SESSION_NAME}:isaaclab".0
tmux select-pane -t "${SESSION_NAME}:isaaclab".1
tmux split-window -v -t "${SESSION_NAME}:isaaclab".1
tmux select-layout -t "${SESSION_NAME}:isaaclab" tiled

# ---- Pane titles (tmux >= 3.2 supports -T) ----
tmux select-pane -t "${SESSION_NAME}:isaaclab".0 -T "clean shell"
tmux select-pane -t "${SESSION_NAME}:isaaclab".1 -T "isaac sim app shell"
tmux select-pane -t "${SESSION_NAME}:isaaclab".2 -T "ROS 2 CLI shell [isaac]"
tmux select-pane -t "${SESSION_NAME}:isaaclab".3 -T "container logs [isaac]"

# --- Window 1: isaaclab (container: isaac-lab-ros2) ---

# Pane A (0): clean shell inside container (std_startup_shell)
send "${SESSION_NAME}:isaaclab.0" \
"docker exec -it $ISAAC_CTR bash 'cd /workspace; exec bash -l'"

# Pane B (1): Isaac Sim shell with bridge libs (PYTHONPATH + LD_LIBRARY_PATH)
send "${SESSION_NAME}:isaaclab.1" \
"docker exec -it $ISAAC_CTR bash 'cd /isaac-sim; source shortcuts.bash; ep311; epld; exec bash -l'"

# Pane C (2): ROS 2 CLI shell (Humble sourced)
send "${SESSION_NAME}:isaaclab.2" \
"docker exec -it $ISAAC_CTR bash 'cd /workspace/ros_ws; source shortcuts.bash; soh; exec bash -l'"

# Pane D (3): container logs
send "${SESSION_NAME}:isaaclab.3" \
"docker exec -it $ISAAC_CTR bash -lc 'source /opt/ros/humble/setup.bash; watch -n1 \"ros2 topic list | sort\"'"

# --- Window 2: moveit (container: moveit_jazzy_cuda_container) ---
tmux new-window -t "$SESSION_NAME" -n moveit
tmux split-window -h -t "${SESSION_NAME}:moveit"
tmux select-pane -t "${SESSION_NAME}:moveit".0
tmux split-window -v -t "${SESSION_NAME}:moveit".0
tmux select-pane -t "${SESSION_NAME}:moveit".1
tmux split-window -v -t "${SESSION_NAME}:moveit".1
tmux select-layout -t "${SESSION_NAME}:moveit" tiled

# Pane titles
tmux select-pane -t "${SESSION_NAME}:moveit".0 -T "moveit shell 0"
tmux select-pane -t "${SESSION_NAME}:moveit".1 -T "moveit shell 1"
tmux select-pane -t "${SESSION_NAME}:moveit".2 -T "ros2 topic list"
tmux select-pane -t "${SESSION_NAME}:moveit".3 -T "container logs [moveit]"

# Pane A (0): interactive MoveIt shell (Jazzy + your workspace)
send "${SESSION_NAME}:moveit.0" \
"docker exec -it $MOVEIT_CTR bash 'cd /ros2_ws; source shortcuts.bash; soj; exec bash -l'"

# Pane B (1): second interactive MoveIt shell
send "${SESSION_NAME}:moveit.1" \
"docker exec -it $MOVEIT_CTR bash 'cd /ros2_ws; source shortcuts.bash; soj; exec bash -l'"

# Pane C (2): continuously list topics
send "${SESSION_NAME}:moveit.2" \
"docker exec -it $MOVEIT_CTR bash -lc 'source /opt/ros/jazzy/setup.bash; watch -n1 \"ros2 topic list | sort\"'"

# Pane D (3): MoveIt container logs
send "${SESSION_NAME}:moveit.3" "docker logs -f $MOVEIT_CTR"

# --- Window 3: host tools ---
tmux new-window -t "$SESSION_NAME" -n host
tmux split-window -h -t "${SESSION_NAME}:host"
tmux select-pane -t "${SESSION_NAME}:host".0
tmux split-window -v -t "${SESSION_NAME}:host".0
tmux select-pane -t "${SESSION_NAME}:host".1
tmux split-window -v -t "${SESSION_NAME}:host".1
tmux select-layout -t "${SESSION_NAME}:host" tiled

# Pane titles
tmux select-pane -t "${SESSION_NAME}:host".0 -T "htop"
tmux select-pane -t "${SESSION_NAME}:host".1 -T "nvidia-smi"
tmux select-pane -t "${SESSION_NAME}:host".2 -T "docker ps watch"
tmux select-pane -t "${SESSION_NAME}:host".3 -T "shell in ~/.localgit [host]"

# Pane A: htop
send "${SESSION_NAME}:host.0" "htop"

# Pane B: nvidia-smi
send "${SESSION_NAME}:host.1" "nvidia-smi -l 1"

# Pane C: docker ps watch
send "${SESSION_NAME}:host.2" "watch -n2 \"docker ps --format 'table {{.Names}}\\t{{.Status}}'\""

# Pane D: host shell in ~/.localgit
send "${SESSION_NAME}:host.3" "cd ~/.localgit; bash -l"

# Focus first window/pane and attach
tmux select-window -t "${SESSION_NAME}:isaaclab"
tmux select-pane -t "${SESSION_NAME}:isaaclab".0
exec tmux attach -t "$SESSION_NAME"