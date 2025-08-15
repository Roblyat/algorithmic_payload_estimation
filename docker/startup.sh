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
ISAAC_ENV="$ROOT/docker/.env.isaacsim"  # our Isaac Sim env file
ISAAC_CTR="isaac-lab-ros2"

# Optional: allow local X clients (for GUIs)
command -v xhost >/dev/null 2>&1 && xhost +local: || true

log "Starting Isaac Lab (ros2)…"
(
  cd "$ISAAC" || { err "Missing: $ISAAC"; exit 1; }
  export COMPOSE_PROJECT_NAME=isaaclab
  ./container.py start ros2 --env-files "$ISAAC_ENV"
)
ok "Isaac Lab container started: $ISAAC_CTR"

log "Copying FastDDS XML to $ISAAC_CTR…"
docker exec -u root "$ISAAC_CTR" bash -lc 'mkdir -p /root/.ros'
docker cp "$ROOT/docker/fastdds.xml" "$ISAAC_CTR":/root/.ros/fastdds.xml
ok "FastDDS XML copied."

log "Starting your MoveIt/ROS2 container…"
(
  cd "$MOVEIT" || { err "Missing: $MOVEIT"; exit 1; }
  export COMPOSE_PROJECT_NAME=moveit
  docker compose -f docker-compose.yaml up -d
)
ok "MoveIt/ROS2 container started."

ok "Done."
printf "  IsaacLab: docker compose -p isaaclab ps\n"
printf "  MoveIt:   docker compose -p moveit ps\n"

# Ensure check script is executable, then run it
chmod +x "$ROOT/docker/check_ros_env.sh" || true
"$ROOT/docker/check_ros_env.sh"