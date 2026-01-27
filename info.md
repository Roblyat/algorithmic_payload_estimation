# ENVIRONMENT SETUP - Key ROS and system variables

xhost +local:docker

vps share: smb://192.168.0.206/share/

## Export APE_REPO
export APE_REPO="$HOME/.localgit/algorithmic_payload_estimation"
export APE_PE="$HOME/.localgit/algorithmic_payload_estimation/payload_estimation"
export APE_DELAN="$HOME/.localgit/algorithmic_payload_estimation/deep_lagrangian_networks"
export APE_PREPROCESS="$HOME/.localgit/algorithmic_payload_estimation/payload_estimation/services/preprocess"
export APE_SHARED="$HOME/.localgit/algorithmic_payload_estimation/payload_estimation/shared"
export APE_EVALUATION="$HOME/.localgit/algorithmic_payload_estimation/payload_estimation/services/evaluation"
export LSTM="$HOME/.localgit/algorithmic_payload_estimation/payload_estimation/services/lstm"

clear folders example: rm -rf -- !(.gitkeep|delan_ur5_tf0p2_vf0p1_dataset)

## Show key env values one by one
echo "ROS_DISTRO=$ROS_DISTRO"
echo "RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
echo "FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-<unset or empty>}"
echo "PYTHONPATH=${PYTHONPATH:-<unset or empty>}"
echo "isaac_sim_package_path=$isaac_sim_package_path"
echo "ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH:-<unset or empty>}"

## Also helpful: confirm Isaac/IsaacLab roots the compose provides
echo "DOCKER_ISAACLAB_PATH=$DOCKER_ISAACLAB_PATH"
echo "DOCKER_ISAACSIM_ROOT_PATH=$DOCKER_ISAACSIM_ROOT_PATH"
echo "HOME=$HOME"
echo "PWD=$PWD"

---

# ISAAC LAUNCH SHELL - Humble

unset PYTHONPATH LD_LIBRARY_PATH COLCON_PREFIX_PATH
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export ROS_DISTRO=humble
export AMENT_PREFIX_PATH=/workspace/ros_ws/install:/isaac-sim/exts/isaacsim.ros2.bridge/humble
export PYTHONPATH=/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib/python3.11/site-packages
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$isaac_sim_package_path/exts/isaacsim.ros2.bridge/humble/lib

---

# ISAAC LAUNCH SHELL - Jazzy


unset PYTHONPATH LD_LIBRARY_PATH COLCON_PREFIX_PATH
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export ROS_DISTRO=humble
export AMENT_PREFIX_PATH=/ros_ws/install:/isaac-sim/exts/isaacsim.ros2.bridge/jazzy
export PYTHONPATH=/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/lib/python3.11/site-packages
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$isaac_sim_package_path/exts/isaacsim.ros2.bridge/jazzy/lib

---

# WORKAROUNDS: Shell A - Build Overlay (OK to source Humble here)

source /opt/ros/humble/setup.bash

mkdir -p /workspace/ros_ws/src
cd /workspace/ros_ws/src

## Bring in the description repo (symlink or copy; symlink is fine)
ln -s /ros_pkgs/Universal_Robots_ROS2_Description .

## Install any deps (usually none for ur_description, but harmless)
cd /workspace/ros_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y

## Build & install only the description package
colcon build --packages-select ur_description --merge-install

## Sanity check the ament index now contains ur_description
test -f install/share/ament_index/resource_index/packages/ur_description && echo "ament index OK"

---

# Launch Isaac 

- (DO NOT source /opt/ros/humble here)

unset PYTHONPATH LD_LIBRARY_PATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export ROS_DISTRO=humble

## Isaac’s own bridge prefix + your overlay
export AMENT_PREFIX_PATH=/workspace/ros_ws/install:/isaac-sim/exts/isaacsim.ros2.bridge/humble

## Make sure Isaac’s py311 rclpy is used (no /opt/ros on PYTHONPATH!)
export PYTHONPATH=/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib/python3.11/site-packages

## (optional but handy if you also want fallback package:// resolution in the *normal* URDF importer)
export ROS_PACKAGE_PATH=/ros_pkgs/Universal_Robots_ROS2_Description/ur_description:\
/ros_pkgs/ur_manipulator_description:\
/ros_pkgs/ur_manipulator_moveit_config${ROS_PACKAGE_PATH:+:$ROS_PACKAGE_PATH}

---

# MOVEIT CONFIG - jazzy moveit_controllers.yaml


-  don’t forget action_ns & default

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

---

# ECHO ENV

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

---

# EXPORTS - Shell B: Launch Isaac (clean env, no /opt/ros sourced)

## 0) Start from a clean slate in this shell only
unset PYTHONPATH
unset LD_LIBRARY_PATH
unset AMENT_PREFIX_PATH
unset COLCON_PREFIX_PATH

## 1) DDS/ROS core
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export ROS_DISTRO=humble

## 2) Ament index for package:// resolution (RobotDescription importer uses this)
export AMENT_PREFIX_PATH=/workspace/ros_ws/install:/isaac-sim/exts/isaacsim.ros2.bridge/humble

## 3) Make Isaac use its own Python 3.11 rclpy (critical!)
export PYTHONPATH=/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib/python3.11/site-packages

## 4) (optional) helps the *normal* URDF importer before the overlay is built
export ROS_PACKAGE_PATH=/workspace/ros_ws/src/Universal_Robots_ROS2_Description/ur_description:\
/workspace/ros_ws/src/ur_manipulator_description/ur_manipulator_description:\
/workspace/ros_ws/src/ur_manipulator_moveit_config/ur3_manipulator_moveit_config

## 5) (optional) hint some tools read
export isaac_sim_package_path=/isaac-sim

## 6) sanity check (should print a valid path; if it errors, build ur_description first)
python3 - <<'PY'
from ament_index_python.packages import get_package_share_directory as g
try:
    print("ur_description share:", g("ur_description"))
except Exception as e:
    print("ament lookup failed:", e)
PY

## 7) launch Isaac Sim
/isaac-sim/isaac-sim.sh