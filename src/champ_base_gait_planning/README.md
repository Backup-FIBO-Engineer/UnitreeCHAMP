# champ_base_gait_planning

CHAMP gait planning + IK/FK for the XGO quadruped (SolidWorks URDF) and
Unitree Go2 (official `go2_description` kinematics).

XGO and Go2 share the same controller/estimator nodes. Each robot has its
own URDF, gait yaml, launch files, and MuJoCo model. Do not mix Go2 LowCmd
with the Unitree Sport API.

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

Lock files (URDF xyz + gait yaml) must match `tools/verify_champ_go2.cpp`:

```bash
python3 tools/verify_go2_gait_yaml.py
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
  -o /tmp/verify_go2_lowcmd tools/verify_go2_lowcmd.cpp src/motor_crc.cpp
/tmp/verify_go2_lowcmd
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

## Run RViz test (Go2)

Kinematic walk only. Does not start MuJoCo or LowCmd.

```bash
ros2 launch champ_base_gait_planning rviz_gait_test_go2.launch.py
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

Walk with teleop. Keep speed at **0.2–0.5 m/s** (`gait.max_linear_velocity_x` is 0.50). Press `i` to go forward. Do not mash `q` — teleop default already 0.5, and 1.5 m/s is silently clamped.

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Regenerate the Go2 MJCF after changing URDF translations:

```bash
python3 tools/generate_go2_mjcf.py
```

## Go2 Sim2Real (Unitree DDS LowCmd)

Needs [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2) so CMake
can build `go2_dds_bridge`. That node talks **native DDS** (`rt/lowcmd`,
`rt/lowstate`), not ROS 2 `unitree_go` msgs and not the Sport API.

1. Ethernet to the robot, Sport / motion services **off** (the bridge calls
   `MotionSwitcher ReleaseMode`).
2. Install unitree_sdk2, then rebuild this package.
3. Launch (replace `eth0` with the NIC that reaches the Go2):

```bash
ros2 launch champ_base_gait_planning go2_sim2real.launch.py network_interface:=eth0
```

The launch binds ROS 2 CycloneDDS to `lo` so it does not share the robot NIC
with unitree_sdk2. After `ReleaseMode`, the bridge holds the measured pose
(`motor_mode` 0x01, CRC on `LowCmd_`), then ramps from that pose to CHAMP
commands over `ramp_sec` at 500 Hz. Measured `LowState_` is published as
`/joint_states`, `/foot_contacts`, and `/imu/data` for odometry.

If CHAMP commands stop, the last pose is held with extra damping.

Do not mix this launch with Sport. It is not started by the RViz launch.

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
- `config/go2_lowcmd.yaml` — Sim2Real kp/kd, ramp, `rt/lowcmd` rate
- `urdf/go2.urdf` — official Unitree description, meshes under `meshes/Go2/`

## Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/cmd_vel` | `geometry_msgs/Twist` | in |
| `/joint_states` | `sensor_msgs/JointState` | out (controller) |
| `/foot_contacts` | `champ_msgs/ContactsStamped` | out (controller) |
| `/cmd_vel/estimated` | `geometry_msgs/Twist` | out (estimator) |
| `/odom/raw` | `nav_msgs/Odometry` | out (estimator) |
| `/foot` | `visualization_msgs/MarkerArray` | out (estimator) |
| `rt/lowcmd` | Unitree `LowCmd_` DDS | out (`go2_dds_bridge`, hardware) |
| `rt/lowstate` | Unitree `LowState_` DDS | in (`go2_dds_bridge`, hardware) |
