#!/bin/bash


cd ~/UnitreeCHAMP
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch champ_base_gait_planning mujoco_sim_b2.launch.py
