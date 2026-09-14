# unitree_rl_deploy

Run a **trained** Go2 or B2 locomotion policy in MuJoCo and on the real robot.
No training code.

```
/cmd_vel  +  IMU  +  joints  +  odom (lin_vel)  ->  policy_runner  ->  /joint_commands
                                                                          |
                              MuJoCo  (sim)                               +-->  unitree_ros2_bridge -> /lowcmd
```

Xbox teleop: `./run_xbox_teleop.sh go2` (or `b2`) publishes `/cmd_vel`.

## Files you put in

| File | Role |
|---|---|
| `config/go2_rl.yaml` / `config/b2_rl.yaml` | Observation layout, `default_angles`, joint order, `action_scale` |
| `policies/` | Your `.pt` (TorchScript or actor `state_dict`) or `.onnx` |
| `urdf/`, `mujoco/`, `config/*_lowcmd.yaml` | Robot model and motor PD |

`joint_names`, `default_angles` and `observation.terms` **must match the env
that produced the checkpoint**. The shipped yaml is the **48-D** actor:

`lin_vel, ang_vel, gravity, command, dof_pos, dof_vel, action`

`lin_vel` is body-frame base velocity (Isaac `base_lin_vel`), scaled by
`lin_vel_scale` (default 2.0). It is **not** filled with zeros.

| Source | Topic | Twist frame | Typical use |
|---|---|---|---|
| MuJoCo | `odom/ground_truth` (`nav_msgs/Odometry`) | body (`lin_vel_frame: body`) | `./run_mujoco_rl.sh` |
| Estimator odom | `odom_topic:=/odom` | body or `lin_vel_frame:=world` | real robot |
| DLS MUSE / HAL | `base_state_topic:=/base_state` | world (rotated here) | `dls2_interface/BaseState` |

If `lin_vel` is in `terms` and odom/`/base_state` is missing or stale, the runner
holds `default_angles` instead of stepping the policy. A 45-D actor is the same
list without `lin_vel` (then odom is unused).

```bash
./run_rl_sim2real.sh go2 eth0 policy:=/abs/path/policy.pt odom_topic:=/odom
./run_rl_sim2real.sh go2 eth0 policy:=/abs/path/policy.pt base_state_topic:=/base_state
```

## Build

Needs `unitree_go` / `unitree_api` so `unitree_ros2_bridge` is compiled.
Policy inference needs **PyTorch** (`.pt`) or **onnxruntime** (`.onnx`).
Empty `policy:=` holds `default_angles` (stand).

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select unitree_rl_deploy --symlink-install
source install/setup.bash
python3 -m pytest src/unitree_rl_deploy/test -q
```

## MuJoCo

```bash
ros2 launch unitree_rl_deploy mujoco_rl.launch.py robot:=go2 policy:=/abs/path/policy.pt
./run_mujoco_rl.sh b2 policy:=/abs/path/policy.pt
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}" -r 10
```

## Real robot

Sport off, NIC toward the robot.

```bash
ros2 launch unitree_rl_deploy unitree_rl_sim2real.launch.py robot:=go2 \
  network_interface:=eth0 policy:=/abs/path/policy.pt odom_topic:=/odom
ros2 launch unitree_rl_deploy unitree_rl_sim2real.launch.py robot:=go2 \
  network_interface:=eth0 policy:=/abs/path/policy.pt base_state_topic:=/base_state
ros2 launch unitree_rl_deploy unitree_rl_sim2real.launch.py robot:=b2 \
  network_interface:=eth0 policy:=/abs/path/policy.pt imu_topic:=/dog_imu_raw
./run_xbox_teleop.sh b2
```

`unitree_ros2_bridge` maps joints **by name**, ramps into the first command,
clamps to URDF limits, and talks `/lowcmd` + `/lowstate`. If the training PD
is not the stand-example gains in `_lowcmd.yaml`, set `kp` / `kd` in `_rl.yaml`.

```bash
ros2 topic echo /rl/observation
ros2 topic echo /rl/action
ros2 topic echo /joint_commands
ros2 topic echo /cmd_vel
```
