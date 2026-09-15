#!/bin/bash
# Sim2Sim MuJoCo for Go2 Rough-Blind.
# Usage: ./run_mujoco_rl.sh [robot] [extra launch args]
#   ./run_mujoco_rl.sh
#   ./run_mujoco_rl.sh go2 policy:=/abs/path/policy.onnx
set -e
cd "$(dirname "$0")"

ROBOT="go2"
if [[ $# -gt 0 && "${1}" != *:=* ]]; then
  ROBOT="${1}"
  shift
fi

ROS_DISTRO="${ROS_DISTRO:-humble}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source install/setup.bash
ros2 launch unitree_rl_deploy mujoco_rl.launch.py "robot:=${ROBOT}" "$@"
