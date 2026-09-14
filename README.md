# UnitreeUniversalCHAMP

CHAMP gait / IK / odometry, MuJoCo simulation and Unitree LowCmd Sim2Real for
Unitree quadrupeds (branch `UnitreeUniversalCHAMP`, merged from `Go2EDU` and
`B2EDU`). The workspace contains:

- `src/champ` — kinematics, leg controller, odometry (headers)
- `src/champ_msgs` — `ContactsStamped`
- `src/champ_base_gait_planning` — ROS 2 nodes, robot descriptions, MuJoCo, Sim2Real bridge (`unitree_ros2_bridge`)

Sim2Real talks to the robot through
[unitree_ros2](https://github.com/unitreerobotics/unitree_ros2): the robot is
a plain participant of the ROS 2 / CycloneDDS graph (`/lowcmd`, `/lowstate`,
`/api/motion_switcher/*`), so there is one DDS stack, no `unitree_sdk2`.

One code base drives every robot. Robot-specific values live only in the files
named after the robot inside `src/champ_base_gait_planning`
(`urdf/<robot>.urdf`, `config/<robot>_{gait,joints,links,sim,lowcmd,body_pose}.yaml`,
`mujoco/<robot>.xml`, `rviz/<robot>_gait.rviz`); the nodes, launch files and
tools read joint names, limits, leg geometry, gains and motor mode from them.
Shipped robots: `go2`, `b2` (both with LowCmd) and `xgo` (sim/RViz only).
See `src/champ_base_gait_planning/README.md` for the file contract and how to
add another Unitree robot without touching code.

Branch `UnitreeUniversalCHAMPBodyPose` adds an IMU-closed body roll/pitch
loop (`body_pose_controller_node`) between the user's `/body_pose` and CHAMP:
the IMU orientation is turned into base roll/pitch and a bounded PI(D)
correction keeps the body at the desired roll/pitch while CHAMP keeps walking
with gait + IK + joint PD unchanged. It runs in MuJoCo for `go2`, `b2`, `xgo`
and on the real `go2` / `b2` (IMU from `/lowstate` or an external IMU topic).

## Build

```bash
source /opt/ros/humble/setup.bash          # or jazzy
colcon build --packages-select champ champ_msgs champ_base_gait_planning
source install/setup.bash
```

Hardware Sim2Real also needs the `unitree_go` / `unitree_api` message
packages of unitree_ros2 so `unitree_ros2_bridge` is compiled (it is skipped
with a CMake warning otherwise). Once, on the PC cabled to the robot:

```bash
sudo apt install ros-$ROS_DISTRO-rmw-cyclonedds-cpp ros-$ROS_DISTRO-rosidl-generator-dds-idl
git clone https://github.com/unitreerobotics/unitree_ros2 ~/unitree_ros2
cd ~/unitree_ros2/cyclonedds_ws && colcon build --packages-select unitree_go unitree_api
```

then `source ~/unitree_ros2/cyclonedds_ws/install/setup.bash` before building
this workspace (`./run_build_ws.sh` does it when `~/unitree_ros2` exists).
Follow the unitree_ros2 README to edit its `setup.sh` (ROS distro, NIC name);
`source ~/unitree_ros2/setup.sh` exports `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
and a `CYCLONEDDS_URI` bound to the robot NIC. `ROS_DOMAIN_ID` must stay 0 /
unset: the robot is on domain 0.

## Verify (no ROS runtime)

```bash
bash src/champ_base_gait_planning/tools/run_offline_checks.sh          # all robots
bash src/champ_base_gait_planning/tools/run_offline_checks.sh go2 b2   # selected robots
```

Per robot: URDF→CHAMP contract, IK/FK, gait at yaml limits, odometry
sign/scale, MuJoCo model vs URDF, MuJoCo standing FK, forward-walk tracking,
(robots with `_lowcmd.yaml`) the Unitree motor map, URDF limit clamp and CRC,
and (robots with `_body_pose.yaml`) the IMU body pose loop: conventions,
CHAMP body-pose sign, the angle limits reachable by IK and inside the URDF
joint limits standing and walking at the gait yaml velocities, slope /
set-point / saturation / release / ramp behaviour with the robot's gains.
`colcon test --packages-select champ_base_gait_planning` runs the same checks.

## Run

RViz kinematic walk:

```bash
ros2 launch champ_base_gait_planning rviz_gait_test.launch.py robot:=b2
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {z: 0.0}}" -r 10
```

MuJoCo:

```bash
ros2 launch champ_base_gait_planning mujoco_sim.launch.py robot:=b2     # or robot:=go2 / robot:=xgo
./run_mujoco.sh b2
```

Teleop: keep the speed below `gait.max_linear_velocity_x` of the robot
(`config/<robot>_gait.yaml`); CHAMP clamps anything above it.

Xbox One S (Model 1708, Bluetooth) — locomotion `/cmd_vel` and body-pose
mode on the same pad (`src/xbox_one_s_teleop`):

```bash
./run_xbox_teleop.sh b2          # or go2 / xgo; do not run keyboard teleop too
# launch log must be "Opened joystick: Xbox ...", not RustDesk
```

LB toggles walk ↔ body pose, hold RB to walk (left stick vx/vy, right stick
yaw), right stick in pose mode rolls/pitches the body, A resets to level.
Pairing, `jstest`, and the button table: `src/xbox_one_s_teleop/README.md`.

IMU body roll/pitch loop (on by default when `config/<robot>_body_pose.yaml`
exists; `body_pose_control:=false` turns it off):

```bash
# stand on a slope: the body stays level while the loop runs (standing and walking)
ros2 launch champ_base_gait_planning mujoco_sim.launch.py robot:=go2 floor_pitch:=0.10 floor_roll:=0.05
# ask for a nose-up of 0.10 rad (RPY 0, -0.10, 0; +pitch = nose down), ramped at desired_rate
ros2 topic pub --once /body_pose geometry_msgs/msg/Pose "{orientation: {y: -0.04998, w: 0.99875}}"
ros2 topic echo /body_pose/measured_rpy      # filtered body roll, pitch (rad), IMU yaw
ros2 topic echo /body_pose/correction        # closed-loop share of the command (rad)
ros2 param set /body_pose_controller_node body_pose.enabled false   # release the correction (ramped), pass /body_pose through
```

Limits (`config/<robot>_body_pose.yaml`, total roll / pitch handed to CHAMP):
Go2 0.25 / 0.20 rad, B2 0.25 / 0.25 rad, XGO 0.10 / 0.05 rad; the correction
is bounded to 0.15 rad (XGO 0.05) and shares that budget. They are set from
the kinematic ceilings measured by `verify_body_pose_controller` while walking
at the gait yaml maximum velocities; see the per-robot table and the Sim2Real
procedure / cautions in `src/champ_base_gait_planning/README.md`.

Real robot (Sport off, NIC toward the robot):

```bash
ros2 launch champ_base_gait_planning unitree_sim2real.launch.py robot:=b2 network_interface:=eth0
ros2 launch champ_base_gait_planning unitree_sim2real.launch.py robot:=go2 network_interface:=eth0
```

`network_interface:=<nic>` sets `RMW_IMPLEMENTATION` / `CYCLONEDDS_URI` for
every node of the launch (same values as unitree_ros2 `setup.sh`); leave it
empty when that `setup.sh` is already sourced. The bridge publishes
`unitree_go/LowCmd` on `/lowcmd` (CRC, motor order FR/FL/RR/RL) with the
`kp` / `kd` / `motor_mode` from `config/<robot>_lowcmd.yaml`, clamps every
joint target to the URDF limits of the selected robot, and reads `/lowstate`.
Stand with Sport first; the bridge releases Sport (`/api/motion_switcher`),
sends no LowCmd while the robot still reports a motion service active, then
holds the pose and ramps into CHAMP over `ramp_sec`. Do not mix with the
Sport API.

The same launch starts the IMU body roll/pitch loop for `go2` / `b2`
(`body_pose_control:=auto|true|false`). By default it reads `/imu/data`, the
`/lowstate` IMU republished by the bridge (~500 Hz); an external fused IMU
works too, e.g. `imu_topic:=/dog_imu_raw_aligned` (best-effort or reliable
publishers both match). The loop only starts once the bridge is on `/lowcmd`
and the IMU is flowing; a body tilt beyond `body_pose.max_error` (not
standing) or an IMU older than `imu_timeout_sec` releases the correction
(ramped at `max_rate`, never a step; the desired pose is ramped at
`desired_rate` as well). Tune `config/<robot>_body_pose.yaml` on the robot with
`/body_pose/measured_rpy` and `/body_pose/correction`; start with the shipped
`kp`/`ki` and `max_correction`, and raise them only once the standing robot
levels on a wedge without hunting.
