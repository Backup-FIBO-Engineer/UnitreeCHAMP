#!/bin/bash
# Sim2Real Go2 Rough-Blind: policy_runner + unitree_ros2_bridge.
# MUSE must run in another terminal:
#   ./run_muse.sh
#   # or: ros2 launch state_estimator state_estimator.launch.py
#
# Usage: ./run_rl_sim2real.sh <network_interface> [extra launch args]
#   ./run_rl_sim2real.sh eth0 policy:=/abs/path/policy.onnx
#   ./run_rl_sim2real.sh go2 eth0 policy:=/abs/path/policy.onnx
set -e
cd "$(dirname "$0")"

if [[ $# -lt 1 ]]; then
  echo "usage: $0 [robot] <network_interface> [launch args]" >&2
  echo "  $0 eth0 policy:=/abs/path/policy.onnx" >&2
  exit 1
fi

ROBOT="go2"
NIC=""
if [[ "${1}" == *:=* ]]; then
  echo "usage: $0 [robot] <network_interface> [launch args]" >&2
  exit 1
fi
if [[ $# -ge 2 && "${2}" != *:=* ]]; then
  ROBOT="${1}"
  NIC="${2}"
  shift 2
else
  NIC="${1}"
  shift
fi

ROS_DISTRO="${ROS_DISTRO:-humble}"
UNITREE_ROS2_DIR="${UNITREE_ROS2_DIR:-$HOME/unitree_ros2}"
MUSE_WS="${MUSE_WS:-$HOME/muse/muse_ws}"

source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [ -f "${UNITREE_ROS2_DIR}/cyclonedds_ws/install/setup.bash" ]; then
  source "${UNITREE_ROS2_DIR}/cyclonedds_ws/install/setup.bash"
fi
if [ -f "${MUSE_WS}/install/setup.bash" ]; then
  source "${MUSE_WS}/install/setup.bash"
  echo "MUSE overlay from ${MUSE_WS} (dls2_interface for /base_state)"
else
  echo "WARNING: MUSE not found at ${MUSE_WS}." >&2
  echo "  Clone https://github.com/iit-DLSLab/muse -b unitree_sdk and build muse_ws." >&2
  echo "  policy_runner needs dls2_interface to subscribe to /base_state." >&2
fi
source install/setup.bash

ros2 launch unitree_rl_deploy unitree_rl_sim2real.launch.py \
  "robot:=${ROBOT}" "network_interface:=${NIC}" "$@"
