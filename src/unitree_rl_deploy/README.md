# unitree_rl_deploy

Run a **trained** Go2 or B2 locomotion policy in MuJoCo and on the real robot.
This package has **no training code**. Drop a checkpoint next to the existing
Sim2Real stack (`unitree_ros2_bridge` + `mujoco_sim.py` in
`champ_base_gait_planning`). CHAMP gait is not started.

```
/cmd_vel  +  IMU  +  joints  ->  policy_runner  ->  /joint_commands
                                                      |
                          MuJoCo  (sim)               +-->  unitree_ros2_bridge -> /lowcmd  (real)
```

Xbox teleop is unchanged: `./run_xbox_teleop.sh go2` (or `b2`) publishes `/cmd_vel`.

## Files you put in

| File | Role |
|---|---|
| `config/go2_rl.yaml` / `config/b2_rl.yaml` | Observation layout, `default_angles`, joint order, `action_scale` |
| `policies/` | Your `.pt` (TorchScript or actor `state_dict`) or `.onnx` |
| `champ_base_gait_planning` URDF / MuJoCo / `_lowcmd.yaml` | Robot model and motor PD |

`joint_names`, `default_angles` and `observation.terms` **must match the env
that produced the checkpoint**. A 45-D actor that was trained as

`ang_vel, gravity, command, dof_pos, dof_vel, action`

loads with the shipped yaml. If your policy used `lin_vel`, a gait clock, or
a stacked history, edit `observation.terms` / `history` only — do not change
the node.

## Build

Needs the same workspace as Sim2Real (`unitree_go` / `unitree_api` sourced so
the bridge is compiled). Policy inference needs **PyTorch** (`.pt`) or
**onnxruntime** (`.onnx`). Empty `policy:=` holds `default_angles` (stand)
so you can test topics without a file.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select champ champ_msgs champ_base_gait_planning unitree_rl_deploy --symlink-install
source install/setup.bash
python3 -m pytest src/unitree_rl_deploy/test -q
```

## MuJoCo

```bash
ros2 launch unitree_rl_deploy mujoco_rl.launch.py robot:=go2 policy:=/abs/path/policy.pt
# or
./run_mujoco_rl.sh b2 policy:=/abs/path/policy.pt
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}" -r 10
```

Without `policy:=` the robot stands at `default_angles` (wiring check).

## Real robot

Sport off, NIC toward the robot, same CycloneDDS rules as CHAMP Sim2Real.
Hold / strap the robot until the policy is tracking.

```bash
ros2 launch unitree_rl_deploy unitree_rl_sim2real.launch.py robot:=go2 \
  network_interface:=eth0 policy:=/abs/path/policy.pt
# B2, external IMU:
ros2 launch unitree_rl_deploy unitree_rl_sim2real.launch.py robot:=b2 \
  network_interface:=eth0 policy:=/abs/path/policy.pt imu_topic:=/dog_imu_raw
./run_xbox_teleop.sh b2
```

`unitree_ros2_bridge` still maps joints **by name**, ramps into the first
command, clamps to URDF limits, and talks `/lowcmd` + `/lowstate`. If the
training PD is not the stand-example gains in `_lowcmd.yaml`, set `kp` / `kd`
in the robot `_rl.yaml`.

Debug:

```bash
ros2 topic echo /rl/observation
ros2 topic echo /rl/action
ros2 topic echo /joint_commands
ros2 topic echo /cmd_vel
```

## What this is not

- Not Isaac Gym / rsl_rl / PPO. Train elsewhere, export the actor, run it here.
- Not CHAMP. Gait launches (`mujoco_sim.launch.py`, `unitree_sim2real.launch.py`)
  stay in `champ_base_gait_planning` if you still want the analytic trot.
