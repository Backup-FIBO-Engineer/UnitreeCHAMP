#!/bin/bash


cd ~/UnitreeCHAMP
rm -rf build install log
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build


