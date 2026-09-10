# Go2CHAMP

CHAMP gait / IK / odometry for Unitree Go2. This branch contains only:

- `src/champ` — kinematics, leg controller, odometry (headers)
- `src/champ_msgs` — `ContactsStamped`
- `src/champ_base_gait_planning` — ROS 2 nodes, Go2 URDF, MuJoCo, Sim2Real DDS

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select champ champ_msgs champ_base_gait_planning
source install/setup.bash
```

Hardware Sim2Real also needs [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2) so `go2_dds_bridge` is compiled.

## Verify (no ROS)

```bash
bash src/champ_base_gait_planning/tools/run_offline_checks.sh
```

This checks URDF→CHAMP xyz, gait yaml lock, IK/FK, gait at yaml limits, odometry sign/scale, MuJoCo standing FK, and a forward-walk tracking check.

## Run

RViz kinematic walk:

```bash
ros2 launch champ_base_gait_planning rviz_gait_test_go2.launch.py
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {z: 0.0}}" -r 10
```

MuJoCo:

```bash
ros2 launch champ_base_gait_planning mujoco_sim_go2.launch.py
```

Teleop: keep speed 0.2–0.5 m/s, press `i` for forward. Do not raise teleop above 0.5 (`q`); CHAMP clamps to `gait.max_linear_velocity_x`.

Real Go2 (Sport off, NIC toward the robot):

```bash
ros2 launch champ_base_gait_planning go2_sim2real.launch.py network_interface:=eth0
```

The bridge publishes Unitree DDS `rt/lowcmd` (`LowCmd_`, CRC, motor order
FR/FL/RR/RL) and reads `rt/lowstate`. Stand with Sport first; the launch
releases Sport and holds pose. Do not mix with the Sport API.
