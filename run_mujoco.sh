#!/bin/bash



source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch champ_base_gait_planning mujoco_sim_go2.launch.py
