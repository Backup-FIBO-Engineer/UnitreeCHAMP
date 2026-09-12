#!/bin/bash
# Build the CHAMP workspace. Sim2Real (unitree_ros2_bridge) needs the unitree_go /
# unitree_api message packages of https://github.com/unitreerobotics/unitree_ros2:
#   git clone https://github.com/unitreerobotics/unitree_ros2 ~/unitree_ros2
#   cd ~/unitree_ros2/cyclonedds_ws && colcon build --packages-select unitree_go unitree_api
# They are sourced from UNITREE_ROS2_DIR (default ~/unitree_ros2) when present;
# otherwise the bridge is skipped with a CMake warning and the rest still builds.

ROS_DISTRO="${ROS_DISTRO:-humble}"
UNITREE_ROS2_DIR="${UNITREE_ROS2_DIR:-$HOME/unitree_ros2}"

cd ~/UnitreeCHAMP
rm -rf build install log
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [ -f "${UNITREE_ROS2_DIR}/cyclonedds_ws/install/setup.bash" ]; then
  source "${UNITREE_ROS2_DIR}/cyclonedds_ws/install/setup.bash"
  echo "unitree_ros2 messages from ${UNITREE_ROS2_DIR}/cyclonedds_ws"
else
  echo "unitree_ros2 not found at ${UNITREE_ROS2_DIR}; unitree_ros2_bridge will be skipped"
fi
# rosdep has no keys for the unitree_ros2 message packages.
rosdep install --from-paths src --ignore-src --rosdistro "${ROS_DISTRO}" -y \
  --skip-keys "unitree_go unitree_api"
colcon build
