#!/bin/bash
set -e  # exit on error

cd "$(dirname "$0")"

# Clean old build
rm -rf build install log

# Source ROS environment
source /opt/ros/humble/setup.bash

UNITREE_ROS2_DIR="${UNITREE_ROS2_DIR:-$HOME/unitree_ros2}"
if [ -f "${UNITREE_ROS2_DIR}/cyclonedds_ws/install/setup.bash" ]; then
  source "${UNITREE_ROS2_DIR}/cyclonedds_ws/install/setup.bash"
  echo "unitree_ros2 messages from ${UNITREE_ROS2_DIR}/cyclonedds_ws"
fi

# Install dependencies
echo "Which script do you want to run?"
echo "1) Without rosdep."
echo "2) With rosdep."
read -p "Enter your choice (1 or 2): " choice
case $choice in
    1)
        echo "Running without rosdep."
        ;;
    2)
        echo "Running with rosdep."
        rosdep install --from-paths src --ignore-src --rosdistro humble -y \
          --skip-keys "unitree_go unitree_api" || true
        ;;
    *)
        echo "Invalid choice. Please enter 1 or 2."
        exit 1
        ;;
esac

colcon build --packages-select unitree_rl_deploy xbox_one_s_teleop --symlink-install
