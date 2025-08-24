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