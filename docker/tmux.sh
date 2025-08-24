#!/usr/bin/env bash
set -euo pipefail

# --- config (change if your container names differ) ---
ISAAC_CTR="${ISAAC_CTR:-isaac-lab-ros2}"
MOVEIT_CTR="${MOVEIT_CTR:-force_estimation_container}"
SESSION_NAME="${SESSION_NAME:-robot}"

# --- sanity checks ---
command -v tmux >/dev/null 2>&1 || { echo "tmux not found. Install tmux first."; exit 1; }
docker ps --format '{{.Names}}' | grep -qx "$ISAAC_CTR"  || { echo "Container not running: $ISAAC_CTR"; exit 1; }
docker ps --format '{{.Names}}' | grep -qx "$MOVEIT_CTR"  || { echo "Container not running: $MOVEIT_CTR"; exit 1; }

# --- attach if session already exists (don’t clobber a running layout) ---
if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
  exec tmux attach -t "$SESSION_NAME"
fi

# Helper to send a command to a pane (pane target like: "robot:isaaclab.0")
send() { tmux send-keys -t "$1" "$2" C-m; }

# Create session with first window
tmux new-session -d -s "$SESSION_NAME" -n isaaclab

# Build 2x2 layout in Window 1 (isaaclab)
# Pane index mapping after splits:
#  0 = left-top (A), 1 = right-top (B), 2 = left-bottom (C), 3 = right-bottom (D)
tmux split-window -h -t "${SESSION_NAME}:isaaclab"
tmux select-pane -t "${SESSION_NAME}:isaaclab".0
tmux split-window -v -t "${SESSION_NAME}:isaaclab".0
tmux select-pane -t "${SESSION_NAME}:isaaclab".1
tmux split-window -v -t "${SESSION_NAME}:isaaclab".1
tmux select-layout -t "${SESSION_NAME}:isaaclab" tiled

# --- Window 1: isaaclab ---
# Pane A (0): interactive shell inside isaac-lab-ros2
send "${SESSION_NAME}:isaaclab.0" \
"docker exec -it $ISAAC_CTR bash -lc 'cd /workspace/isaaclab; export PYTHONPATH=\"\"; export AMENT_PREFIX_PATH=/opt/ros/humble; exec bash -l'"

# Pane B (1): isaac-sim work dir, bridge path appended, clean PYTHONPATH, AMENT_PREFIX_PATH set
send "${SESSION_NAME}:isaaclab.1" \
"docker exec -it $ISAAC_CTR bash -lc 'cd /isaac-sim; export PYTHONPATH=\"\"; export AMENT_PREFIX_PATH=/opt/ros/humble; BRIDGE_LIB=\"/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib\"; case \":\$LD_LIBRARY_PATH:\" in *\":\$BRIDGE_LIB:\"*) ;; *) export LD_LIBRARY_PATH=\"\${LD_LIBRARY_PATH:+\$LD_LIBRARY_PATH:}\$BRIDGE_LIB\" ;; esac; exec bash -l'"

# Pane C (2): docker logs -f isaac-lab-ros2
send "${SESSION_NAME}:isaaclab.2" "docker logs -f $ISAAC_CTR"

# Pane D (3): ROS 2 CLI shell inside isaac container (Humble sourced if you want; keep PYTHONPATH clean)
send "${SESSION_NAME}:isaaclab.3" \
"docker exec -it $ISAAC_CTR bash -lc 'export PYTHONPATH=\"\"; export AMENT_PREFIX_PATH=/opt/ros/humble; source /opt/ros/humble/setup.bash; cd /isaac-sim; exec bash -l'"

# --- Window 2: moveit ---
tmux new-window -t "$SESSION_NAME" -n moveit
tmux split-window -h -t "${SESSION_NAME}:moveit"
tmux select-pane -t "${SESSION_NAME}:moveit".0
tmux split-window -v -t "${SESSION_NAME}:moveit".0
tmux select-pane -t "${SESSION_NAME}:moveit".1
tmux split-window -v -t "${SESSION_NAME}:moveit".1
tmux select-layout -t "${SESSION_NAME}:moveit" tiled

# Pane A (0): interactive shell in MoveIt container with ROS Jazzy sourced
send "${SESSION_NAME}:moveit.0" \
"docker exec -it $MOVEIT_CTR bash -lc 'source /opt/ros/jazzy/setup.bash; cd /ros2_ws; exec bash -l'"

# Pane B (1): logs
send "${SESSION_NAME}:moveit.1" "docker logs -f $MOVEIT_CTR"

# Pane C (2): watch topics
send "${SESSION_NAME}:moveit.2" \
"docker exec -it $MOVEIT_CTR bash -lc 'source /opt/ros/jazzy/setup.bash; watch -n1 \"ros2 topic list | sort\"'"

# Pane D (3): second interactive shell
send "${SESSION_NAME}:moveit.3" \
"docker exec -it $MOVEIT_CTR bash -lc 'source /opt/ros/jazzy/setup.bash; cd /ros2_ws; exec bash -l'"

# --- Window 3: host tools ---
tmux new-window -t "$SESSION_NAME" -n host
tmux split-window -h -t "${SESSION_NAME}:host"
tmux select-pane -t "${SESSION_NAME}:host".0
tmux split-window -v -t "${SESSION_NAME}:host".0
tmux select-pane -t "${SESSION_NAME}:host".1
tmux split-window -v -t "${SESSION_NAME}:host".1
tmux select-layout -t "${SESSION_NAME}:host" tiled

# Pane A: htop
send "${SESSION_NAME}:host.0" "htop"

# Pane B: nvidia-smi
send "${SESSION_NAME}:host.1" "nvidia-smi -l 1"

# Pane C: docker ps watch
send "${SESSION_NAME}:host.2" "watch -n2 \"docker ps --format 'table {{.Names}}\t{{.Status}}'\""

# Pane D: host shell in ~/.localgit
send "${SESSION_NAME}:host.3" "cd ~/.localgit; bash -l"

# Focus first window/pane and attach
tmux select-window -t "${SESSION_NAME}:isaaclab"
tmux select-pane -t "${SESSION_NAME}:isaaclab".0
exec tmux attach -t "$SESSION_NAME"