#!/bin/bash
# Launch MUSE state estimator (not vendored in this repo).
# Required for Go2 Rough-Blind Sim2Real: publishes dls2_interface/BaseState
# on /base_state from IMU + joints + foot force on /lowstate.
#
#   git clone https://github.com/iit-DLSLab/muse.git -b unitree_sdk
#   cd muse && conda env create -f environment.yml && conda activate muse-ros2
#   cd muse_ws && vcs import src < muse.repos && colcon build --symlink-install
#
# Usage:
#   ./run_muse.sh
#   MUSE_WS=/path/to/muse/muse_ws ./run_muse.sh
set -e
cd "$(dirname "$0")"

ROS_DISTRO="${ROS_DISTRO:-humble}"
UNITREE_ROS2_DIR="${UNITREE_ROS2_DIR:-$HOME/unitree_ros2}"
MUSE_WS="${MUSE_WS:-$HOME/muse/muse_ws}"

if [ ! -f "${MUSE_WS}/install/setup.bash" ]; then
  cat >&2 <<EOF
MUSE is not built at ${MUSE_WS}

Install (branch unitree_sdk, tested by DLS on Go2):

  git clone https://github.com/iit-DLSLab/muse.git -b unitree_sdk
  cd muse
  conda env create -f environment.yml
  conda activate muse-ros2
  cd muse_ws
  vcs import src < muse.repos
  colcon build --symlink-install

Then:

  source ${MUSE_WS}/install/setup.bash
  ros2 launch state_estimator state_estimator.launch.py

Or set MUSE_WS to your muse_ws path and rerun $0
EOF
  exit 1
fi

source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [ -f "${UNITREE_ROS2_DIR}/cyclonedds_ws/install/setup.bash" ]; then
  source "${UNITREE_ROS2_DIR}/cyclonedds_ws/install/setup.bash"
fi
# Same CycloneDDS NIC as Sim2Real so /lowstate from the robot reaches MUSE.
if [ -n "${NETWORK_INTERFACE:-}" ]; then
  export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
  export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${NETWORK_INTERFACE}\" priority=\"default\" multicast=\"default\"/></Interfaces></General></Domain></CycloneDDS>"
  echo "CycloneDDS bound to ${NETWORK_INTERFACE}"
fi
source "${MUSE_WS}/install/setup.bash"
echo "Launching MUSE state_estimator (needs /lowstate from the Go2)."
exec ros2 launch state_estimator state_estimator.launch.py "$@"
