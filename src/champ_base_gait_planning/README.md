# champ_base_gait_planning

CHAMP gait planning + IK/FK for the XGO quadruped (SolidWorks URDF),
Unitree Go2 (official `go2_description` kinematics) and Unitree B2 (official
`b2_description` kinematics from `unitree_ros`).

XGO, Go2 and B2 share the same controller/estimator nodes. Each robot has its
own URDF, gait yaml, launch files, and MuJoCo model. Go2 and B2 share one
`unitree_dds_bridge` (`robot: go2|b2` in `config/<robot>_lowcmd.yaml`). Do not
mix Unitree LowCmd with the Unitree Sport API.

## Pipeline

1. **cmd_vel → joints**: `quadruped_controller_node`
   - Gait planning (`LegController::velocityCommand`)
   - Inverse kinematics (`Kinematics::inverse`)
   - Publishes `/joint_states`, `/foot_contacts`

2. **joints → cmd_vel**: `state_estimation_node`
   - Forward kinematics (`QuadrupedLeg::foot_from_base`)
   - Velocity estimation (`Odometry::getVelocities`)
   - Publishes `/odom/raw`, `/cmd_vel/estimated`, `/foot`

## Verify kinematics / gait / odom (no ROS)

These checks use the same joint xyz as `urdf/xgo_rviz.xacro` (CHAMP's loader
sums translations; that is exact here because every joint `rpy` is 0).

```bash
python3 tools/verify_urdf_champ_contract.py \
  urdf/xgo_rviz.xacro config/xgo_links.yaml config/xgo_joints.yaml

python3 tools/validate_mujoco_against_urdf.py urdf/xgo_rviz.xacro mujoco/xgo.xml
python3 tools/verify_mujoco_physics.py mujoco/xgo.xml

g++ -std=c++17 -O2 \
  -I ../champ/include/champ \
  -o /tmp/verify_champ_xgo tools/verify_champ_xgo.cpp
/tmp/verify_champ_xgo
```

After a colcon build of this package, `colcon test --packages-select champ_base_gait_planning` runs the same checks.

Lock files (URDF xyz + gait yaml) must match `tools/verify_champ_go2.cpp` /
`tools/verify_champ_b2.cpp`:

```bash
python3 tools/verify_gait_yaml.py go2
python3 tools/verify_gait_yaml.py b2
bash tools/run_offline_checks.sh
```

Go2 (official URDF: hip `+X`, thigh/calf `+Y`, `rpy=0`, root link `base`):

```bash
python3 tools/verify_urdf_champ_contract.py \
  urdf/go2.urdf config/go2_links.yaml config/go2_joints.yaml --preset go2

python3 tools/validate_mujoco_against_urdf.py \
  urdf/go2.urdf mujoco/go2.xml --robot go2
python3 tools/verify_mujoco_physics.py mujoco/go2.xml

g++ -std=c++17 -O2 \
  -I ../champ/include/champ \
  -o /tmp/verify_champ_go2 tools/verify_champ_go2.cpp
/tmp/verify_champ_go2

g++ -std=c++17 -O2 \
  -I include \
  -o /tmp/verify_unitree_lowcmd tools/verify_unitree_lowcmd.cpp src/motor_crc.cpp
/tmp/verify_unitree_lowcmd
```

B2 (official URDF: hip `+X`, thigh/calf `+Y`, `rpy=0`, root link `base_link`,
reach `0.35 + 0.35 = 0.70 m`, limits hip `±0.87`, thigh `[-0.94, 4.69]`,
calf `[-2.82, -0.43]`):

```bash
python3 tools/verify_urdf_champ_contract.py \
  urdf/b2.urdf config/b2_links.yaml config/b2_joints.yaml --preset b2

python3 tools/validate_mujoco_against_urdf.py \
  urdf/b2.urdf mujoco/b2.xml --robot b2
python3 tools/verify_mujoco_physics.py mujoco/b2.xml
python3 tools/verify_mujoco_walk.py --robot b2 0.35

g++ -std=c++17 -O2 \
  -I ../champ/include/champ \
  -o /tmp/verify_champ_b2 tools/verify_champ_b2.cpp
/tmp/verify_champ_b2
```

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select champ champ_msgs champ_base_gait_planning
source install/setup.bash
```

## Run RViz test (XGO)

```bash
ros2 launch champ_base_gait_planning rviz_gait_test.launch.py
```

## Run RViz test (Go2 / B2)

Kinematic walk only. Does not start MuJoCo or LowCmd.

```bash
ros2 launch champ_base_gait_planning rviz_gait_test_go2.launch.py
ros2 launch champ_base_gait_planning rviz_gait_test_b2.launch.py
```

Send velocity commands:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10
```

Compare estimated velocity:

```bash
ros2 topic echo /cmd_vel/estimated
```

## MuJoCo

XGO:

```bash
ros2 launch champ_base_gait_planning mujoco_sim.launch.py
```

Go2 (`mujoco/go2.xml`, root link `base`):

```bash
ros2 launch champ_base_gait_planning mujoco_sim_go2.launch.py
```

B2 (`mujoco/b2.xml`, root link `base_link`, ~74.6 kg incl. rotors/head/tail,
position actuators kp 1000 / kv 20, hip kp 500, force limits 200/200/320 Nm):

```bash
ros2 launch champ_base_gait_planning mujoco_sim_b2.launch.py
```

Walk with teleop. Keep speed at **0.2–0.5 m/s** on Go2 (`gait.max_linear_velocity_x` is 0.50) and **0.2–0.6 m/s** on B2 (0.60). Press `i` to go forward. Do not mash `q` — teleop default already 0.5, and 1.5 m/s is silently clamped.

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Regenerate the Go2/B2 MJCF after changing URDF translations:

```bash
python3 tools/generate_unitree_mjcf.py        # both
python3 tools/generate_unitree_mjcf.py b2
```

## Go2 / B2 Sim2Real (Unitree DDS LowCmd)

Needs [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2) so CMake
can build `unitree_dds_bridge`. That node talks **native DDS** (`rt/lowcmd`,
`rt/lowstate`, the `unitree_go` `LowCmd_`/`LowState_` IDL that Go2 and B2
share), not ROS 2 `unitree_go` msgs and not the Sport API.

1. Ethernet to the robot, Sport / motion services **off** (the bridge calls
   `MotionSwitcher ReleaseMode`).
2. Install unitree_sdk2, then rebuild this package.
3. Launch (replace `eth0` with the NIC that reaches the robot):

```bash
ros2 launch champ_base_gait_planning go2_sim2real.launch.py network_interface:=eth0
ros2 launch champ_base_gait_planning b2_sim2real.launch.py network_interface:=eth0
```

The launch binds ROS 2 CycloneDDS to `lo` so it does not share the robot NIC
with unitree_sdk2. After `ReleaseMode`, the bridge holds the measured pose
(CRC on `LowCmd_`), then ramps from that pose to CHAMP commands over
`ramp_sec` at 500 Hz. Every joint target is clamped to the URDF limits of the
selected robot before it is written to `rt/lowcmd`. Measured `LowState_` is
published as `/joint_states`, `/foot_contacts`, and `/imu/data` for odometry.

Per-robot values come from `config/<robot>_lowcmd.yaml` (`robot:` selects the
limit table and the default `motor_mode`):

| | Go2 | B2 |
|---|---|---|
| `motor_mode` | `1` (0x01) | `10` (0x0A) |
| `kp` / `kd` | 40 / 1 | 1000 / 10 (unitree_sdk2 `b2_stand_example`) |
| `ramp_sec` | 2.0 | 3.0 |
| `contact_force_threshold` | 20 | 40 |

If CHAMP commands stop, the last pose is held with extra damping.

Do not mix this launch with Sport. It is not started by the RViz launch.

unitree_sdk2 ships its own CycloneDDS (`libddsc.so.0`, `libddscxx.so.0`) with
the same soname as the ROS 2 `rmw_cyclonedds` copy. The bridge binary is
linked with a `DT_RPATH` to the unitree_sdk2 lib directory so its SDK side
always gets the matching CycloneDDS regardless of `LD_LIBRARY_PATH` order
(the mismatch aborted with `free(): invalid pointer` as soon as `rt/lowstate`
was discovered). Check with
`ldd $(ros2 pkg prefix champ_base_gait_planning)/lib/champ_base_gait_planning/unitree_dds_bridge | grep ddsc`;
both entries must point at the unitree_sdk2 install.

## Config

XGO:

- `config/xgo_rviz_gait.yaml` — gait parameters (nominal_height, swing_height, etc.)
- `config/xgo_joints.yaml` — joint name mapping for CHAMP
- `config/xgo_links.yaml` — link chain for URDF kinematics
- `urdf/xgo_rviz.xacro` — XGO robot description

Go2:

- `config/go2_gait.yaml` — gait (nominal_height 0.30 m, swing 0.08 m, max vx 0.50)
- `config/go2_joints.yaml` — `FL_*` / `FR_*` / `RL_*` / `RR_*`
- `config/go2_links.yaml` — root link `base`
- `config/go2_lowcmd.yaml` — Sim2Real `robot: go2`, kp/kd, ramp, `rt/lowcmd` rate
- `urdf/go2.urdf` — official Unitree description, meshes under `meshes/Go2/`

B2:

- `config/b2_gait.yaml` — gait (nominal_height 0.50 m, swing 0.10 m, stance 0.30 s, max vx 0.60)
- `config/b2_joints.yaml` — `FL_*` / `FR_*` / `RL_*` / `RR_*` (same names as Go2)
- `config/b2_links.yaml` — root link `base_link`
- `config/b2_lowcmd.yaml` — Sim2Real `robot: b2`, kp 1000 / kd 10, mode 0x0A, ramp 3 s
- `urdf/b2.urdf` — official `unitree_ros` `b2_description`, meshes under `meshes/B2/`
  (the four identical `*_calf.dae` are shared as one `calf.dae`)

## Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/cmd_vel` | `geometry_msgs/Twist` | in |
| `/joint_states` | `sensor_msgs/JointState` | out (controller) |
| `/foot_contacts` | `champ_msgs/ContactsStamped` | out (controller) |
| `/cmd_vel/estimated` | `geometry_msgs/Twist` | out (estimator) |
| `/odom/raw` | `nav_msgs/Odometry` | out (estimator) |
| `/foot` | `visualization_msgs/MarkerArray` | out (estimator) |
| `rt/lowcmd` | Unitree `LowCmd_` DDS | out (`unitree_dds_bridge`, hardware) |
| `rt/lowstate` | Unitree `LowState_` DDS | in (`unitree_dds_bridge`, hardware) |
