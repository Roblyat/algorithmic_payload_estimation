#!/usr/bin/env bash
set -euo pipefail

# ---- pretty logging ----
is_tty() { [[ -t 1 ]]; }
if is_tty; then
  C_RESET='\e[0m'; C_INFO='\e[36m'; C_OK='\e[32m'; C_WARN='\e[33m'; C_ERR='\e[31m'
else
  C_RESET=''; C_INFO=''; C_OK=''; C_WARN=''; C_ERR=''
fi
tag()   { printf "${C_INFO}[CHECK] %s${C_RESET}\n" "$*"; }
good()  { printf "${C_OK}[CHECK] ✔ %s${C_RESET}\n" "$*"; }
bad()   { printf "${C_ERR}[CHECK] ✖ %s${C_RESET}\n" "$*"; }
note()  { printf "${C_WARN}[CHECK] ! %s${C_RESET}\n" "$*"; }

# expected values
declare -A EXP_ISAAC=(
  [FASTRTPS_DEFAULT_PROFILES_FILE]="/root/.ros/fastdds.xml"
  [RMW_IMPLEMENTATION]="rmw_fastrtps_cpp"
)
declare -A EXP_MOVEIT=(
  [ROS_DISTRO]="jazzy"
  [RMW_IMPLEMENTATION]="rmw_fastrtps_cpp"
  [FASTRTPS_DEFAULT_PROFILES_FILE]="/home/robat/.ros/fastdds.xml"
)

print_env_block() {
  local ctr="$1"
  docker exec "$ctr" bash -lc \
    "env | grep -E 'ROS_DISTRO|RMW_IMPLEMENTATION|FASTRTPS_DEFAULT_PROFILES_FILE|LD_LIBRARY_PATH|isaac_sim_package_path' || echo 'No env found'"
}

read_var() {
  local ctr="$1" var="$2"
  docker exec "$ctr" bash -lc "printenv $var || true"
}

check_ctr() {
  local ctr="$1"; shift
  local -n EXPECTED="$1"

  tag "=== $ctr ==="
  print_env_block "$ctr"

  local fail=0
  for var in "${!EXPECTED[@]}"; do
    local exp="${EXPECTED[$var]}"
    local value
    value="$(read_var "$ctr" "$var")"

    if [[ -z "$value" ]]; then
      bad "$var is NOT set (expected: $exp)"; fail=1; continue
    fi
    if [[ "$value" == "$exp" ]]; then
      good "$var = $value"
    else
      bad "$var = $value  (expected: $exp)"; fail=1
    fi
  done

  # Only annotate LD_LIBRARY_PATH (no validation)
  local ld
  ld="$(read_var "$ctr" "LD_LIBRARY_PATH")"
  [[ -n "$ld" ]] && note "$ctr LD_LIBRARY_PATH: $ld" || note "$ctr LD_LIBRARY_PATH is empty"

  return "$fail"
}

overall=0

# ensure containers are up before checking
for c in isaac-lab-ros2 force_estimation_container; do
  if ! docker ps --format '{{.Names}}' | grep -qx "$c"; then
    bad "Container not running: $c"; overall=1
  fi
done

if docker ps --format '{{.Names}}' | grep -qx isaac-lab-ros2; then
  check_ctr isaac-lab-ros2 EXP_ISAAC || overall=1
fi
if docker ps --format '{{.Names}}' | grep -qx force_estimation_container; then
  check_ctr force_estimation_container EXP_MOVEIT || overall=1
fi

(( overall == 0 )) && good "All environment checks passed." || { bad "One or more environment checks failed."; exit 1; }