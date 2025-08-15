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