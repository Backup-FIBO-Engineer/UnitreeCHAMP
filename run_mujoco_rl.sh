#!/bin/bash
# Usage: ./run_mujoco_rl.sh <robot> [extra launch args]
#   ./run_mujoco_rl.sh go2
#   ./run_mujoco_rl.sh b2 policy:=/path/to/policy.pt
set -e
cd "$(dirname "$0")"

ROBOT="${1:?usage: $0 <robot> [launch args]  (robot = go2 or b2)}"
shift

ROS_DISTRO="${ROS_DISTRO:-humble}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source install/setup.bash
ros2 launch unitree_rl_deploy mujoco_rl.launch.py "robot:=${ROBOT}" "$@"
