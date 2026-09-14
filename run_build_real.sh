#!/bin/bash
set -e  # exit on error

# Go to workspace
cd ~/UnitreeCHAMP


# Clean old build
rm -rf build install log


# Build cyclonedds only
colcon build --packages-select cyclonedds


# Source ROS environment
source /opt/ros/humble/setup.bash



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
        rosdep install --from-paths src --ignore-src --rosdistro humble -y
        ;;
    *)
        echo "Invalid choice. Please enter 1 or 2."
        exit 1
        ;;
esac


# Build everything
colcon build

