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
ISAAC_ENV="$ROOT/docker/env.isaacsim"
ISAAC_CTR="isaac-lab-ros2"
ISAAC_CMP="$ROOT/docker/isaac.compose.yaml"

# Export for compose substitution
export APE_REPO="${APE_REPO:-$ROOT}"

# Which MoveIt stack to start: humble (default) or jazzy
TARGET="${1:-jazzy}"

case "${TARGET}" in
  humble|Humble) SERVICE="moveit_humble" ;;
  jazzy|Jazzy)   SERVICE="moveit_cuda_jazzy" ;;
  *)
    err "Usage: $0 [humble|jazzy]"
    exit 2
    ;;
esac

# ---- Start Isaac (base profile) ----
log "Starting Isaac Lab (ros2)…"
(
  cd "$ISAAC" || { err "Missing: $ISAAC"; exit 1; }
  export COMPOSE_PROJECT_NAME=isaaclab
  ./container.py start ros2 --files "$ISAAC_CMP" --env-files "$ISAAC_ENV"
)
ok "Isaac Lab container started: $ISAAC_CTR"

# ---- Start MoveIt container (selected service) ----
log "Starting your MoveIt/ROS2 container ($SERVICE)…"
(
  cd "$MOVEIT" || { err "Missing: $MOVEIT"; exit 1; }
  export COMPOSE_PROJECT_NAME=moveit
  docker compose -f docker-compose.yaml up -d "$SERVICE"
)
ok "MoveIt/ROS2 container started: $SERVICE"

# ---- Post info ----
ok "Done."
printf "  IsaacLab: docker compose -p isaaclab ps\n"
printf "  MoveIt:   docker compose -p moveit ps\n"

# Env check
chmod +x "$ROOT/docker/check_ros_env.sh" || true
"$ROOT/docker/check_ros_env.sh"

# tmux (optional)
chmod +x "$ROOT/docker/tmux.sh" || true
"$ROOT/docker/tmux.sh"