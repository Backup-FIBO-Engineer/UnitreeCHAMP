# UnitreeRLDeploy

Run a **trained** Unitree Go2 or B2 locomotion policy in MuJoCo and on the
real robot over [`unitree_ros2`](https://github.com/unitreerobotics/unitree_ros2)
(`/lowcmd`, `/lowstate`). Policy deploy only; there is **no RL training**
in this repository.

```
/cmd_vel  +  IMU  +  joints  ->  policy_runner  ->  /joint_commands
                                                      |
                          MuJoCo  (sim)               +-->  unitree_ros2_bridge -> /lowcmd
```

Packages:

- `src/unitree_rl_deploy` — policy runner, MuJoCo, `/lowcmd` bridge, Go2/B2 URDF
- `src/xbox_one_s_teleop` — Xbox One S `/cmd_vel`
- `src/unitree_interfaces` — `unitree_go` / `unitree_api` messages (or source them from unitree_ros2)

Put your actor in `src/unitree_rl_deploy/policies/` or pass `policy:=/path/to/file`
(TorchScript `.pt`, actor `state_dict`, or ONNX). Observation layout lives in
`config/go2_rl.yaml` / `config/b2_rl.yaml` and must match training.

## Build

```bash
source /opt/ros/humble/setup.bash
# hardware: also source unitree_ros2/cyclonedds_ws so the bridge is compiled
colcon build --packages-select unitree_rl_deploy xbox_one_s_teleop --symlink-install
source install/setup.bash
python3 -m pytest src/unitree_rl_deploy/test -q
```

Hardware also needs `unitree_go` / `unitree_api`. `./run_build_ws.sh` sources
`~/unitree_ros2` when present.

## MuJoCo

```bash
./run_mujoco_rl.sh go2 policy:=/abs/path/policy.pt
./run_mujoco_rl.sh b2  policy:=/abs/path/policy.pt
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}" -r 10
```

Without `policy:=` the robot holds `default_angles` (stand) so you can check topics.

## Real robot

Sport off, NIC toward the robot, CycloneDDS on domain 0.

```bash
./run_rl_sim2real.sh go2 eth0 policy:=/abs/path/go2.pt
./run_rl_sim2real.sh b2 eth0 policy:=/abs/path/b2.pt imu_topic:=/dog_imu_raw
./run_xbox_teleop.sh b2
```

The bridge maps joints by name, ramps into the first command, clamps to URDF
limits, and talks `/lowcmd` + `/lowstate`. Match training PD with `kp`/`kd` in
the robot `_rl.yaml` if it is not the stand-example gains in `_lowcmd.yaml`.

Details: `src/unitree_rl_deploy/README.md`.
