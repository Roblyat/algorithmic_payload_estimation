#!/usr/bin/env bash
set -euo pipefail

# ---- pretty logging ----
is_tty() { [[ -t 1 ]]; }
if is_tty; then
  C_RESET='\e[0m'; C_INFO='\e[36m'; C_OK='\e[32m'; C_WARN='\e[33m'; C_ERR='\e[31m'
else
  C_RESET=''; C_INFO=''; C_OK=''; C_WARN=''; C_ERR=''
fi
log()   { printf "${C_INFO}[STARTUP] %s${C_RESET}\n" "$*"; }
ok()    { printf "${C_OK}[STARTUP] %s${C_RESET}\n" "$*"; }
warn()  { printf "${C_WARN}[STARTUP] %s${C_RESET}\n" "$*"; }
err()   { printf "${C_ERR}[STARTUP] %s${C_RESET}\n" "$*"; }

# Paths
ROOT="$HOME/.localgit/algorithmic_payload_estimation"
ISAAC="$ROOT/IsaacLab/docker"
MOVEIT="$ROOT/docker"
ISAAC_ENV="$ROOT/docker/.env.isaacsim"
ISAAC_CTR="isaac-lab-ros"
ISAAC_CMP="$ROOT/docker/ipc-host.yaml"
WS_REPO="/workspace/ros_ws/IsaacSim-ros_workspaces"   # container path

# Export for compose substitution
export APE_REPO="${APE_REPO:-$ROOT}"

# ---- Start Isaac (base profile) ----
log "Starting Isaac Lab (ros)…"
(
  cd "$ISAAC" || { err "Missing: $ISAAC"; exit 1; }
  export COMPOSE_PROJECT_NAME=isaaclab
  ./container.py start ros2 --files "$ISAAC_CMP" --env-files "$ISAAC_ENV"
)
ok "Isaac Lab container started: $ISAAC_CTR"

# ---- Start MoveIt container ----
log "Starting your MoveIt/ROS2 container…"
(
  cd "$MOVEIT" || { err "Missing: $MOVEIT"; exit 1; }
  export COMPOSE_PROJECT_NAME=moveit
  docker compose -f docker-compose.yaml up -d
)
ok "MoveIt/ROS2 container started."

# ---- Post info ----
ok "Done."
printf "  IsaacLab: docker compose -p isaaclab ps\n"
printf "  MoveIt:   docker compose -p moveit ps\n"

# Env check
# chmod +x "$ROOT/docker/check_ros_env.sh" || true
# "$ROOT/docker/check_ros_env.sh"

# tmux (optional)
# chmod +x "$ROOT/docker/tmux.sh" || true
# "$ROOT/docker/tmux.sh"