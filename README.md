# To Do
- remove robotiq & universal_robot submodule
- remove manipulator_description & manipulator_moveit_config
- remove hmi & payload_estimation 
- add ros2 submodules
    - Universal_Robots_ROS2_Driver
    - IsaacLab
    - rbyt_robotiq
    - Universal_Robots_ROS2_Description
## Create ros2 amend_cmake pkgs and:
- rework robotiq submodule
- rework manipulator_description & manipulator_moveit_config
[- rework hmi (no prio, in case as submodule) & payload_estimation (payload_estimation as submodule)]

Disabling the ROS Bridge in isaac-sim.sh

To disable the ROS bridge, use the following steps:

    Open the file located at ~/isaacsim/apps/isaacsim.exp.full.kit.

    Find the line isaac.startup.ros_bridge_extension = "isaacsim.ros2.bridge".

    Change it to isaac.startup.ros_bridge_extension = "" to disable the ROS 2 bridge.

    Save and close the file.

# if anyone wants to bake anything
ok "Isaac Lab container started: $ISAAC_CTR"

log "Injecting Humble overrides into $ISAAC_CTR…"
docker exec -u root "$ISAAC_CTR" bash -lc '
cat >/root/.bashrc_isaac_humble <<EOF
# Isaac Sim ROS 2 override to use internal Humble stack
export isaac_sim_package_path=/isaac-sim
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# Prepend internal Humble ROS 2 libs so they win over /opt/ros/humble
case ":${LD_LIBRARY_PATH:-}:" in
  *":/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib:"*) ;;
  *) export LD_LIBRARY_PATH="/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib:${LD_LIBRARY_PATH:-}";;
esac
# Use UDP transport profile for cross-container Fast DDS discovery
export FASTRTPS_DEFAULT_PROFILES_FILE=/root/.ros/fastdds.xml
EOF

# also install a profile.d file so login shells pick it up
cat >/etc/profile.d/zz-isaacsim-humble.sh <<EOF
export isaac_sim_package_path=/isaac-sim
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
case ":\${LD_LIBRARY_PATH:-}:" in
  *":/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib:"*) ;;
  *) export LD_LIBRARY_PATH="/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib:\${LD_LIBRARY_PATH:-}";;
esac
export FASTRTPS_DEFAULT_PROFILES_FILE=/root/.ros/fastdds.xml
EOF
chmod 644 /etc/profile.d/zz-isaacsim-humble.sh

grep -qF ".bashrc_isaac_humble" /root/.bashrc || echo "[ -f /root/.bashrc_isaac_humble ] && source /root/.bashrc_isaac_humble" >> /root/.bashrc
'
ok "Humble overrides in place."


# issac env option
isaac_sim_package_path=/isaac-sim
ROS_DISTRO=humble
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib
FASTRTPS_DEFAULT_PROFILES_FILE=/root/.ros/fastdds.xml
ROS_DOMAIN_ID=0

#check
2) Avoid Humble↔︎Jazzy DDS wire-compat weirdness

Isaac Sim 5.0’s ROS2 bridge is built around Humble; your other stack is Jazzy. The backtrace hits rmw_dds_common internal discovery types — those can change between distros. Two robust options:

Option A (recommended): switch both to CycloneDDS



# std isaaclab container setup
root@robat-tower:/workspace/isaaclab# printf "RMW_IMPLEMENTATION=%s\nFASTRTPS_DEFAULT_PROFILES_FILE=%s\nROS_DOMAIN_ID=%s\n" \
  "${RMW_IMPLEMENTATION:-<unset>}" \
  "${FASTRTPS_DEFAULT_PROFILES_FILE:-<unset>}" \
  "${ROS_DOMAIN_ID:-<unset>}"
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
FASTRTPS_DEFAULT_PROFILES_FILE=/root/.ros/fastdds.xml
ROS_DOMAIN_ID=<unset>

root@robat-tower:/workspace/isaaclab# ls -d /isaac-sim/exts/isaacsim.ros2.bridge/humble/lib 2>/dev/null || echo "missing"
/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib

robat@robat-tower:~$ docker inspect -f 'network_mode={{.HostConfig.NetworkMode}} ipc={{.HostConfig.IpcMode}}' isaac-lab-ros2
network_mode=host ipc=private



export APE_REPO=/home/robat/.localgit/algorithmic_payload_estimation



##########################################################################

ISAACSIM START
robat@robat-tower:~$ docker exec -it isaac-lab-base bash
root@robat-tower:/workspace/isaaclab# unset PYTHONPATH
root@robat-tower:/workspace/isaaclab# export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$isaac_sim_package_path/exts/isaacsim.ros2.bridge/humble/lib
root@robat-tower:/workspace/isaaclab# export isaac_sim_package_path=$HOME/isaacsim
root@robat-tower:/workspace/isaaclab# export ROS_DISTRO=humble
root@robat-tower:/workspace/isaaclab# echo "AMENT_PREFIX_PATH=$AMENT_PREFIX_PATH"
AMENT_PREFIX_PATH=/opt/ros/humble
root@robat-tower:/workspace/isaaclab# echo "CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH"
CMAKE_PREFIX_PATH=
root@robat-tower:/workspace/isaaclab# echo "LD_LIBRARY_PATH=$LD_LIBRARY_PATH"
LD_LIBRARY_PATH=/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib
root@robat-tower:/workspace/isaaclab# 


CHECK
echo "AMENT_PREFIX_PATH=$AMENT_PREFIX_PATH"       # should include your humble_ws install and /opt/ros/humble
echo "CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH"       # same as AMENT
echo "LD_LIBRARY_PATH=$LD_LIBRARY_PATH"           # no /isaac-sim/... for ROS CLI


ROSTOPIC

# 1) Clean Python path to avoid pollution
unset PYTHONPATH

# 2) Keep ONLY ROS/system libs in the loader path (drop the Isaac path)
export LD_LIBRARY_PATH="/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/usr/lib/x86_64-linux-gnu"

# 3) Source ROS + your overlay
source /opt/ros/humble/setup.bash
source /workspace/IsaacSim-ros_workspaces/humble_ws/install/setup.bash

apt-get install -y ros-humble-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 4) Try again
ros2 topic list


# Show key env values one by one
echo "ROS_DISTRO=$ROS_DISTRO"
echo "RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
echo "FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-<unset or empty>}"
echo "PYTHONPATH=${PYTHONPATH:-<unset or empty>}"
echo "isaac_sim_package_path=$isaac_sim_package_path"
echo "ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH:-<unset or empty>}"

# Also helpful: confirm Isaac/IsaacLab roots the compose provides
echo "DOCKER_ISAACLAB_PATH=$DOCKER_ISAACLAB_PATH"
echo "DOCKER_ISAACSIM_ROOT_PATH=$DOCKER_ISAACSIM_ROOT_PATH"
echo "HOME=$HOME"
echo "PWD=$PWD"


# Isaac launch shell humble
unset PYTHONPATH LD_LIBRARY_PATH COLCON_PREFIX_PATH
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export ROS_DISTRO=humble
export AMENT_PREFIX_PATH=/workspace/ros_ws/install:/isaac-sim/exts/isaacsim.ros2.bridge/humble
export PYTHONPATH=/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib/python3.11/site-packages
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$isaac_sim_package_path/exts/isaacsim.ros2.bridge/humble/lib


# Isaac launch shell jazzy
unset PYTHONPATH LD_LIBRARY_PATH COLCON_PREFIX_PATH
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export ROS_DISTRO=humble
export AMENT_PREFIX_PATH=/ros_ws/install:/isaac-sim/exts/isaacsim.ros2.bridge/jazzy
export PYTHONPATH=/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/lib/python3.11/site-packages
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$isaac_sim_package_path/exts/isaacsim.ros2.bridge/jazzy/lib





# workarounds:
# SHELL A — build overlay (ok to source Humble here)
source /opt/ros/humble/setup.bash

mkdir -p /workspace/ros_ws/src
cd /workspace/ros_ws/src

# Bring in the description repo (symlink or copy; symlink is fine)
ln -s /ros_pkgs/Universal_Robots_ROS2_Description .

# Install any deps (usually none for ur_description, but harmless)
cd /workspace/ros_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build & install only the description package
colcon build --packages-select ur_description --merge-install

# Sanity check the ament index now contains ur_description
test -f install/share/ament_index/resource_index/packages/ur_description && echo "ament index OK"

# -----------------------------------------------------------------------------------------------------------------------

# SHELL B — launch Isaac (DO NOT source /opt/ros/humble here)
unset PYTHONPATH LD_LIBRARY_PATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export ROS_DISTRO=humble

# Isaac’s own bridge prefix + your overlay
export AMENT_PREFIX_PATH=/workspace/ros_ws/install:/isaac-sim/exts/isaacsim.ros2.bridge/humble

# Make sure Isaac’s py311 rclpy is used (no /opt/ros on PYTHONPATH!)
export PYTHONPATH=/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib/python3.11/site-packages

# (optional but handy if you also want fallback package:// resolution in the *normal* URDF importer)
export ROS_PACKAGE_PATH=/ros_pkgs/Universal_Robots_ROS2_Description/ur_description:\
/ros_pkgs/ur_manipulator_description:\
/ros_pkgs/ur_manipulator_moveit_config${ROS_PACKAGE_PATH:+:$ROS_PACKAGE_PATH}

# Start Isaac Sim and enable: ROS 2 Bridge + ROS 2 URDF




# CHECK, MAY KETCHUP
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:+$LD_LIBRARY_PATH:}${isaac_sim_package_path:-/isaac-sim}/exts/isaacsim.ros2.bridge/humble/lib"


# jazzy moveit config "moveit_controllers.yaml"

# MoveIt uses this configuration for controller management

moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager

moveit_simple_controller_manager:
  controller_names:
    - ur3_manipulator_controller

  ur3_manipulator_controller:
    type: FollowJointTrajectory
    action_ns: follow_joint_trajectory
    default: true
    joints:
      - ur3_shoulder_pan_joint
      - ur3_shoulder_lift_joint
      - ur3_elbow_joint
      - ur3_wrist_1_joint
      - ur3_wrist_2_joint
      - ur3_wrist_3_joint


# ECHOS
echo "---- ENVIRONMENT ----"
echo "ROS_DISTRO=$ROS_DISTRO"
echo "RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
echo "FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-<unset>}"
echo
echo "AMENT_PREFIX_PATH=$AMENT_PREFIX_PATH"
echo "PYTHONPATH=$PYTHONPATH"
echo "ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH"
echo "isaac_sim_package_path=$isaac_sim_package_path"
echo
echo "---- PATHS ----"
ls -ld /isaac-sim || echo "missing: /isaac-sim"
ls -ld /workspace/ros_ws || echo "missing: /workspace/ros_ws"
ls -ld /workspace/ros_ws/src || echo "missing: /workspace/ros_ws/src"
ls -ld /workspace/usd || echo "missing: /workspace/usd"
ls -ld /isaac-sim/exts/isaacsim.ros2.bridge/humble || echo "missing: bridge"
ls -ld /isaac-sim/exts/isaacsim.ros2.bridge/humble/lib/python3.11/site-packages || echo "missing: site-packages"


# EXPORTS
# --- Shell B: launch Isaac (clean env, no /opt/ros sourced) ---

# 0) start from a clean slate in this shell only
unset PYTHONPATH
unset LD_LIBRARY_PATH
unset AMENT_PREFIX_PATH
unset COLCON_PREFIX_PATH

# 1) DDS/ROS core
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export ROS_DISTRO=humble

# 2) Ament index for package:// resolution (RobotDescription importer uses this)
export AMENT_PREFIX_PATH=/workspace/ros_ws/install:/isaac-sim/exts/isaacsim.ros2.bridge/humble

# 3) Make Isaac use its own Python 3.11 rclpy (critical!)
export PYTHONPATH=/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib/python3.11/site-packages

# 4) (optional) helps the *normal* URDF importer before the overlay is built
export ROS_PACKAGE_PATH=/workspace/ros_ws/src/Universal_Robots_ROS2_Description/ur_description:\
/workspace/ros_ws/src/ur_manipulator_description/ur_manipulator_description:\
/workspace/ros_ws/src/ur_manipulator_moveit_config/ur3_manipulator_moveit_config

# 5) (optional) hint some tools read
export isaac_sim_package_path=/isaac-sim

# 6) sanity check (should print a valid path; if it errors, build ur_description first)
python3 - <<'PY'
from ament_index_python.packages import get_package_share_directory as g
try:
    print("ur_description share:", g("ur_description"))
except Exception as e:
    print("ament lookup failed:", e)
PY

# 7) launch Isaac Sim
/isaac-sim/isaac-sim.sh
