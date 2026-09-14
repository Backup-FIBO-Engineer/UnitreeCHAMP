#!/bin/bash
# Usage: ./run_rl_sim2real.sh <robot> <nic> [extra launch args]
#   ./run_rl_sim2real.sh go2 eth0 policy:=/path/to/policy.pt
#   ./run_rl_sim2real.sh b2 eth0 policy:=/path/to/policy.pt imu_topic:=/dog_imu_raw
set -e
cd "$(dirname "$0")"

ROBOT="${1:?usage: $0 <robot> <network_interface> [launch args]}"
NIC="${2:?usage: $0 <robot> <network_interface> [launch args]}"
shift 2

ROS_DISTRO="${ROS_DISTRO:-humble}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source install/setup.bash
ros2 launch unitree_rl_deploy unitree_rl_sim2real.launch.py \
  "robot:=${ROBOT}" "network_interface:=${NIC}" "$@"
