# unitree_rl_deploy

Run a **trained** Go2 or B2 DLS Flat / Rough-Blind locomotion policy in MuJoCo
and on the real robot. No training code. Vision / 3D-camera policies are not
wired yet.

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
that produced the checkpoint**. The shipped yaml is the DLS **260-D** actor
(52-D × history 5), same for Go2 and B2, Flat and Rough-Blind:

`lin_vel, ang_vel, gravity, command, dof_pos, dof_vel, last_action, gait_clock`

All scales are **1.0**. Commands are raw `[vx, vy, yaw_rate]` (training samples
about ±0.8 / ±0.25 / ±0.5). Joint / action order is DLS **hip-group**:

```
FL_hip FR_hip RL_hip RR_hip | FL_thigh FR_thigh RL_thigh RR_thigh | FL_calf FR_calf RL_calf RR_calf
```

Policy action `i` maps to legs `FL=[0,4,8], FR=[1,5,9], RL=[2,6,10], RR=[3,7,11]`.
`default_angles` are Isaac init_state: hips 0, thighs 0.9, calves −1.8.
`action_scale=0.5`, clip ±3, action filter α=0.8 on the **joint targets only**
(`last_action` in the observation is the unfiltered clipped policy output).

`lin_vel` is body-frame **COM** velocity (Isaac `root_lin_vel_b`). It is **not**
filled with zeros. The runner rotates world-frame odometry into the body frame,
then adds `ω × r_com` using the URDF base inertial origin.

| Source | Topic | Twist frame | Typical use |
|---|---|---|---|
| MuJoCo | `odom/ground_truth` (`nav_msgs/Odometry`) | body (`lin_vel_frame: body`) | `./run_mujoco_rl.sh` |
| Estimator odom | `odom_topic:=/odom` | body or `lin_vel_frame:=world` | real robot (sim2real default) |
| DLS MUSE / HAL | `base_state_topic:=/base_state` | world (rotated here) | `dls2_interface/BaseState` |

If `lin_vel` is in `terms` and odom/`/base_state` is missing or stale, the runner
holds `default_angles` instead of stepping the policy.

```bash
./run_rl_sim2real.sh go2 eth0 policy:=/abs/path/policy.pt
./run_rl_sim2real.sh go2 eth0 policy:=/abs/path/policy.pt odom_topic:=/odom
./run_rl_sim2real.sh go2 eth0 policy:=/abs/path/policy.pt base_state_topic:=/base_state
```

The actor file must take **260** floats and emit **12** actions (DLS ONNX
`policy.onnx` or an exported 260-D TorchScript). Empty `policy:=` holds
`default_angles` (stand).

## Build

Needs `unitree_go` / `unitree_api` so `unitree_ros2_bridge` is compiled.
Policy inference needs **PyTorch** (`.pt`) or **onnxruntime** (`.onnx`).

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

Position-actuator PD in `mujoco/<robot>.xml` matches DLS walking gains
(Go2 20/2, B2 100/5).

## Real robot

Sport off, NIC toward the robot. You need a body-velocity estimator
(`/odom` or DLS `/base_state`). Without it the policy never leaves the
default pose.

```bash
ros2 launch unitree_rl_deploy unitree_rl_sim2real.launch.py robot:=go2 \
  network_interface:=eth0 policy:=/abs/path/policy.pt
ros2 launch unitree_rl_deploy unitree_rl_sim2real.launch.py robot:=go2 \
  network_interface:=eth0 policy:=/abs/path/policy.pt base_state_topic:=/base_state
ros2 launch unitree_rl_deploy unitree_rl_sim2real.launch.py robot:=b2 \
  network_interface:=eth0 policy:=/abs/path/policy.pt imu_topic:=/dog_imu_raw
./run_xbox_teleop.sh b2
```

`unitree_ros2_bridge` maps joints **by name**, ramps into the first command,
clamps to URDF limits, and talks `/lowcmd` + `/lowstate`. `kp` / `kd` in
`_rl.yaml` override the stand-example gains in `_lowcmd.yaml` (Go2 20/2,
B2 100/5).

```bash
ros2 topic echo /rl/observation
ros2 topic echo /rl/action
ros2 topic echo /joint_commands
ros2 topic echo /cmd_vel
```
