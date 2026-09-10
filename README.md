# B2CHAMP

CHAMP gait / IK / odometry for Unitree B2 (branch `B2EDU`, converted from the
Go2 branch `Go2EDU`; the Go2 files stay in the package). This branch contains only:

- `src/champ` — kinematics, leg controller, odometry (headers)
- `src/champ_msgs` — `ContactsStamped`
- `src/champ_base_gait_planning` — ROS 2 nodes, B2/Go2 URDF, MuJoCo, Sim2Real DDS

B2 numbers (official `b2_description`): hip `(±0.3285, ±0.072, 0)`, thigh
`0.11973`, calf/foot `0.35 + 0.35 = 0.70 m` reach, root link `base_link`,
joint limits hip `±0.87`, thigh `[-0.94, 4.69]`, calf `[-2.82, -0.43]`.
CHAMP stands at `nominal_height 0.50` (thigh `0.775`, calf `-1.550`).

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select champ champ_msgs champ_base_gait_planning
source install/setup.bash
```

Hardware Sim2Real also needs [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2) so `unitree_dds_bridge` is compiled.

## Verify (no ROS)

```bash
bash src/champ_base_gait_planning/tools/run_offline_checks.sh
```

This checks URDF→CHAMP xyz, gait yaml lock, IK/FK, gait at yaml limits, odometry sign/scale, MuJoCo standing FK, and a forward-walk tracking check for both B2 and Go2.

## Run

RViz kinematic walk:

```bash
ros2 launch champ_base_gait_planning rviz_gait_test_b2.launch.py
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {z: 0.0}}" -r 10
```

MuJoCo:

```bash
ros2 launch champ_base_gait_planning mujoco_sim_b2.launch.py
```

Teleop: keep speed 0.2–0.6 m/s, press `i` for forward. CHAMP clamps to
`gait.max_linear_velocity_x` (0.60 for B2).

Real B2 (Sport off, NIC toward the robot):

```bash
ros2 launch champ_base_gait_planning b2_sim2real.launch.py network_interface:=eth0
```

The bridge publishes Unitree DDS `rt/lowcmd` (`LowCmd_`, CRC, motor order
FR/FL/RR/RL, motor mode `0x0A`, kp 1000 / kd 10 as in the unitree_sdk2
`b2_stand_example`) and reads `rt/lowstate`. Joint targets are clamped to the
B2 URDF limits before they leave the bridge. Stand with Sport first; the launch
releases Sport and holds pose, then ramps into CHAMP over 3 s. Do not mix with
the Sport API.

Go2 launches (`*_go2.launch.py`, `go2_sim2real.launch.py`) are unchanged and use
the same `unitree_dds_bridge` with `config/go2_lowcmd.yaml` (`robot: go2`).
