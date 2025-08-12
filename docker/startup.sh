#!/usr/bin/env bash
set -euo pipefail

# Paths
ROOT="$HOME/.localgit/algorithmic_payload_estimation"
ISAAC="$ROOT/IsaacLab/docker"
MOVEIT="$ROOT/docker"

# Optional: allow local X clients (for GUIs); comment out if you don't need it
command -v xhost >/dev/null 2>&1 && xhost +local: || true

echo "[+] Starting Isaac Lab (ros2)..."
(
  cd "$ISAAC" || { echo "Missing: $ISAAC"; exit 1; }
  # Separate compose project so it won't prune your other containers
  export COMPOSE_PROJECT_NAME=isaaclab
  ./container.py start ros2
)

echo "[+] Starting your MoveIt/ROS2 container..."
(
  cd "$MOVEIT" || { echo "Missing: $MOVEIT"; exit 1; }
  export COMPOSE_PROJECT_NAME=moveit
  docker compose -f docker-compose.moveit.yaml up -d
)

echo "[✓] Done."
echo "  IsaacLab: docker compose -p isaaclab ps"
echo "  MoveIt:   docker compose -p moveit ps"





# COPY startup.sh /usr/local/bin/startup.sh
# RUN chmod +x /usr/local/bin/startup.sh
# ENTRYPOINT ["/usr/local/bin/startup.sh"]
