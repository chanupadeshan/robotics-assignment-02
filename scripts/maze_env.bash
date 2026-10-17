# Source this file in every terminal: source scripts/maze_env.bash
maze_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
if [ -f "$maze_workspace/install/local_setup.bash" ]; then
  source "$maze_workspace/install/local_setup.bash"
fi
export GZ_PARTITION=robotics_test
export GZ_IP=127.0.0.1
export TURTLEBOT3_MODEL=burger
export GZ_SIM_RESOURCE_PATH="/opt/ros/jazzy/share/turtlebot3_gazebo/models:${GZ_SIM_RESOURCE_PATH:-}"
export ROS_LOG_DIR="$maze_workspace/log/ros"
export GZ_HOMEDIR="$maze_workspace/log/gazebo"
unset maze_workspace
