#!/bin/bash
# Usage: ./run_xbox_teleop.sh [robot] [extra launch args]
#   ./run_xbox_teleop.sh go2
#   ./run_xbox_teleop.sh b2
#   ./run_xbox_teleop.sh b2 device_id:=1
#   ./run_xbox_teleop.sh b2 device_name:="Xbox Wireless Controller"
#   ./run_xbox_teleop.sh b2 invert_vx:=true   # if stick-forward publishes linear.x < 0
set -e
cd "$(dirname "$0")"

ROBOT="${1:-go2}"
if [[ "${ROBOT}" != *:=* ]]; then
  shift || true
else
  ROBOT="go2"
fi

ROS_DISTRO="${ROS_DISTRO:-humble}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source install/setup.bash
ros2 launch xbox_one_s_teleop teleop.launch.py "robot:=${ROBOT}" "$@"
