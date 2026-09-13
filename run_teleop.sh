#!/bin/bash
# Usage: ./run_teleop.sh
# Publishes /cmd_vel. Keep this in its own terminal; body_pose commands go in
# another shell that has also sourced ROS + install/setup.bash.
set -e
cd "$(dirname "$0")"

ROS_DISTRO="${ROS_DISTRO:-humble}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard "$@"
