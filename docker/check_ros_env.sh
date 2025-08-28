#!/usr/bin/env bash
set -euo pipefail

# ---------------- pretty output ----------------
is_tty() { [[ -t 1 ]]; }
if is_tty; then
  C_RESET='\e[0m'; C_INFO='\e[36m'; C_OK='\e[32m'; C_WARN='\e[33m'; C_ERR='\e[31m'
else
  C_RESET=''; C_INFO=''; C_OK=''; C_WARN=''; C_ERR=''
fi
info() { printf "${C_INFO}[CHECK] %s${C_RESET}\n" "$*"; }
ok()   { printf "${C_OK}[PASS] %s${C_RESET}\n" "$*"; }
warn() { printf "${C_WARN}[WARN] %s${C_RESET}\n" "$*"; }
err()  { printf "${C_ERR}[FAIL] %s${C_RESET}\n" "$*"; }

FAILS=0
fail() { err "$1"; FAILS=$((FAILS+1)); }

# ---------------- container names (adjust if needed) ----------------
ISAAC_CTR="${ISAAC_CTR:-isaac-lab-ros2}"
MOVEIT_CTR="${MOVEIT_CTR:-moveit_jazzy_cuda_container}"  # change if your MoveIt container has a different name

# ---------------- helpers ----------------
# get_var <container> <VAR>  => prints one of: __UNSET__ | __EMPTY__ | actual_value
get_var() {
  local ctr="$1" var="$2"
  docker exec "$ctr" bash -lc '
    v="'"$var"'"
    if [ -z "${!v+x}" ]; then echo __UNSET__;
    elif [ -z "${!v}" ]; then echo __EMPTY__;
    else printf "%s" "${!v}"; fi
  ' 2>/dev/null || { echo "__UNSET__"; return 0; }
}

# expect_exact <who> <container> <VAR> <expected_value>
expect_exact() {
  local who="$1" ctr="$2" var="$3" exp="$4"
  local got; got="$(get_var "$ctr" "$var")"
  if [[ "$got" == "$exp" ]]; then
    ok "$who: $var == $exp"
  else
    fail "$who: $var expected '$exp' but got '$got'"
  fi
}

# expect_unset_or_empty <who> <container> <VAR>
expect_unset_or_empty() {
  local who="$1" ctr="$2" var="$3"
  local got; got="$(get_var "$ctr" "$var")"
  if [[ "$got" == "__UNSET__" || "$got" == "__EMPTY__" ]]; then
    ok "$who: $var is unset/empty"
  else
    fail "$who: $var expected unset/empty but got '$got'"
  fi
}

# expect_path_contains <who> <container> <VAR> <needle_path>
expect_path_contains() {
  local who="$1" ctr="$2" var="$3" needle="$4"
  local got; got="$(get_var "$ctr" "$var")"
  if [[ "$got" == "__UNSET__" || "$got" == "__EMPTY__" ]]; then
    fail "$who: $var is unset/empty; expected to contain '$needle'"
    return
  fi
  IFS=':' read -r -a paths <<< "$got"
  local found=0
  for p in "${paths[@]}"; do
    if [[ "$p" == "$needle" ]]; then found=1; break; fi
  done
  if (( found )); then
    ok "$who: $var contains '$needle'"
  else
    fail "$who: $var does not contain '$needle' (got: $got)"
  fi
}

# path_exists <who> <container> <path>
path_exists() {
  local who="$1" ctr="$2" p="$3"
  if docker exec "$ctr" bash -lc "test -d '$p'"; then
    ok "$who: path exists: $p"
  else
    fail "$who: path missing: $p"
  fi
}

# ---------------- run checks ----------------
info "Checking containers are running…"
if ! docker ps --format '{{.Names}}' | grep -qx "$ISAAC_CTR"; then fail "Container not running: $ISAAC_CTR"; fi
if ! docker ps --format '{{.Names}}' | grep -qx "$MOVEIT_CTR"; then fail "Container not running: $MOVEIT_CTR"; fi

# Isaac expectations
ISAAC_WHO="isaac"
ISAAC_PREFIX_BRIDGE="/isaac-sim/exts/isaacsim.ros2.bridge/humble"
ISAAC_PY311_SITEPKG="${ISAAC_PREFIX_BRIDGE}/lib/python3.11/site-packages"
WS="/workspace/ros_ws"

info "Validating Isaac container env (${ISAAC_CTR})…"
# exact values
expect_exact "$ISAAC_WHO" "$ISAAC_CTR" "isaac_sim_package_path" "/isaac-sim"
expect_exact "$ISAAC_WHO" "$ISAAC_CTR" "ROS_DISTRO" "humble"
expect_exact "$ISAAC_WHO" "$ISAAC_CTR" "RMW_IMPLEMENTATION" "rmw_cyclonedds_cpp"
expect_exact "$ISAAC_WHO" "$ISAAC_CTR" "ROS_DOMAIN_ID" "0"
# unset/empty
expect_unset_or_empty "$ISAAC_WHO" "$ISAAC_CTR" "FASTRTPS_DEFAULT_PROFILES_FILE"

# AMENT_PREFIX_PATH must include overlay + Isaac bridge
expect_path_contains "$ISAAC_WHO" "$ISAAC_CTR" "AMENT_PREFIX_PATH" "${WS}/install"
expect_path_contains "$ISAAC_WHO" "$ISAAC_CTR" "AMENT_PREFIX_PATH" "${ISAAC_PREFIX_BRIDGE}"

# PYTHONPATH must include Isaac’s py311 site-packages
expect_path_contains "$ISAAC_WHO" "$ISAAC_CTR" "PYTHONPATH" "${ISAAC_PY311_SITEPKG}"

# ROS_PACKAGE_PATH must include your package roots in the workspace
expect_path_contains "$ISAAC_WHO" "$ISAAC_CTR" "ROS_PACKAGE_PATH" "${WS}/src/Universal_Robots_ROS2_Description"
expect_path_contains "$ISAAC_WHO" "$ISAAC_CTR" "ROS_PACKAGE_PATH" "${WS}/src/manipulator_description/ur_manipulator_description"
expect_path_contains "$ISAAC_WHO" "$ISAAC_CTR" "ROS_PACKAGE_PATH" "${WS}/src/manipulator_moveit_config/ur3_manipulator_moveit_config"

# path presence (mounts and bridge)
path_exists "$ISAAC_WHO" "$ISAAC_CTR" "/isaac-sim"
path_exists "$ISAAC_WHO" "$ISAAC_CTR" "${WS}"
path_exists "$ISAAC_WHO" "$ISAAC_CTR" "${WS}/src"
path_exists "$ISAAC_WHO" "$ISAAC_CTR" "/workspace/usd"
path_exists "$ISAAC_WHO" "$ISAAC_CTR" "${ISAAC_PREFIX_BRIDGE}"

# extra introspection (non-fatal): show Python executables and search paths
info "Isaac Python info (shell python3 and Isaac kit python)"
docker exec "$ISAAC_CTR" bash -lc 'which python3 || true'
docker exec "$ISAAC_CTR" bash -lc "/isaac-sim/kit/python/bin/python3 -c 'import sys; print(\"PY EXE:\", sys.executable); print(\"--- sys.path ---\"); [print(p) for p in sys.path]' || true"

# show full env paths (useful for quick eyeballing)
docker exec "$ISAAC_CTR" bash -lc 'echo AMENT_PREFIX_PATH=$AMENT_PREFIX_PATH'
docker exec "$ISAAC_CTR" bash -lc 'echo PYTHONPATH=$PYTHONPATH'
docker exec "$ISAAC_CTR" bash -lc 'echo ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH'
docker exec "$ISAAC_CTR" bash -lc 'echo FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-<unset>}'


# MoveIt expectations (no Isaac bridge checks here)
MOVEIT_WHO="moveit"
MOVEIT_WS="/ros2_ws"

info "Validating MoveIt container env (${MOVEIT_CTR})…"
expect_exact "$MOVEIT_WHO" "$MOVEIT_CTR" "ROS_DISTRO" "jazzy"
expect_exact "$MOVEIT_WHO" "$MOVEIT_CTR" "RMW_IMPLEMENTATION" "rmw_cyclonedds_cpp"
expect_exact "$MOVEIT_WHO" "$MOVEIT_CTR" "ROS_DOMAIN_ID" "0"
expect_unset_or_empty "$MOVEIT_WHO" "$MOVEIT_CTR" "FASTRTPS_DEFAULT_PROFILES_FILE"

# Workspace presence (useful sanity, but no Isaac-specific paths)
path_exists "$MOVEIT_WHO" "$MOVEIT_CTR" "${MOVEIT_WS}"
path_exists "$MOVEIT_WHO" "$MOVEIT_CTR" "${MOVEIT_WS}/src"

# Assert DDS modes on Isaac
info "Inspecting Docker modes (Isaac)…"
docker inspect -f 'network={{.HostConfig.NetworkMode}} ipc={{.HostConfig.IpcMode}}' "$ISAAC_CTR" \
  | grep -q 'network=host ipc=host' \
  && ok "isaac: network=host, ipc=host" \
  || fail "isaac: expected network=host and ipc=host"

# ---------------- summary ----------------
if (( FAILS == 0 )); then
  ok "All checks passed."
  exit 0
else
  err "$FAILS check(s) failed."
  exit 1
fi