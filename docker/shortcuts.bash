# git shortcuts
alias gco='git checkout'
alias gcm='git commit -m'
alias gpl='git pull'
alias gps='git push'
alias gst='git status'
alias gsu='git submodule update --init --recursive'
alias gss='git submodule sync'
alias gsa='git submodule add'

# docker shortcuts
alias dcu='docker-compose up'
alias dcd='docker-compose down'
alias dcs='docker-compose ps'
alias dcl='docker-compose logs'

alias dps='docker ps'
alias dpsa='docker ps -a'
alias dil='docker image list'

# ros2 shortcuts
alias soh='source /opt/ros/humble/setup.bash'
alias soj='source /opt/ros/jazzy/setup.bash'
alias si='source install/setup.bash'

alias cb='colcon build'
alias cbsi='colcon build --symlink-install'
alias cbm='colcon build --merge-install'
alias cbps='colcon build --packages-select'

alias rtl='ros2 topic list'
alias rte='ros2 topic echo'
alias rti='ros2 topic info'
alias rnl='ros2 node list'
alias rni='ros2 node info'
alias rla='ros2 launch'

# shell setup shortcuts
alias eape='export APE_REPO=/home/robat/.localgit/algorithmic_payload_estimation'
alias ep311='export PYTHONPATH=/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib/python3.11/site-packages'
alias eld='export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$isaac_sim_package_path/exts/isaacsim.ros2.bridge/humble/lib'