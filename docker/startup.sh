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
ISAAC_ENV="$ROOT/docker/.env.isaacsim"   #custom Isaac env file
ISAAC_CTR="isaac-lab-ros2"
ISAAC_CMP="$ROOT/docker/ipc-host.yaml"   #custom Isaac compose setup"

# Optional: allow local X clients (for GUIs)
command -v xhost >/dev/null 2>&1 && xhost +local: || true

log "Starting Isaac Lab (ros2)…"
(
  cd "$ISAAC" || { err "Missing: $ISAAC"; exit 1; }
  export COMPOSE_PROJECT_NAME=isaaclab
  ./container.py start ros2 --files "$ISAAC_CMP" --env-files "$ISAAC_ENV"
)
ok "Isaac Lab container started: $ISAAC_CTR"

# --- Launch Isaac Lab with Cyclone DDS (override only for this process) ---
# log "Launching Isaac Lab app with Cyclone DDS (unset FASTRTPS, set RMW=rmw_cyclonedds_cpp, ROS_DOMAIN_ID=0)…"
# docker exec -d "$ISAAC_CTR" bash -lc '
#   env -u FASTRTPS_DEFAULT_PROFILES_FILE \
#       RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
#       ROS_DOMAIN_ID=0 \
#       bash -lc '"'"'
#         printf "[Cyclone launch] RMW_IMPLEMENTATION=%s\n" "${RMW_IMPLEMENTATION:-<unset>}"
#         printf "[Cyclone launch] FASTRTPS_DEFAULT_PROFILES_FILE=%s\n" "${FASTRTPS_DEFAULT_PROFILES_FILE:-<unset>}"
#         printf "[Cyclone launch] ROS_DOMAIN_ID=%s\n" "${ROS_DOMAIN_ID:-<unset>}"
#         isaaclab
#       '"'"'
# '
# ok "Isaac Lab launched with Cyclone (detached). Use 'docker logs -f $ISAAC_CTR' to watch output."

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

# Run env check (your script)
chmod +x "$ROOT/docker/check_ros_env.sh" || true
"$ROOT/docker/check_ros_env.sh"