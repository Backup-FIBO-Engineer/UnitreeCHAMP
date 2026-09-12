#!/bin/bash
# Usage: ./run_mujoco.sh <robot> [extra launch args]
#   ./run_mujoco.sh b2
#   ./run_mujoco.sh go2 headless:=true
set -e
cd "$(dirname "$0")"

ROBOT="${1:?usage: $0 <robot> [launch args]  (robot = stem of src/champ_base_gait_planning/urdf/<robot>.urdf|.xacro)}"
shift

ROS_DISTRO="${ROS_DISTRO:-humble}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source install/setup.bash
ros2 launch champ_base_gait_planning mujoco_sim.launch.py "robot:=${ROBOT}" "$@"
