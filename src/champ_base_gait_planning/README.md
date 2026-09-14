# champ_base_gait_planning

CHAMP gait planning + IK/FK + odometry for quadrupeds, with a MuJoCo
simulator and a Unitree `/lowcmd` / `/lowstate` bridge over
[unitree_ros2](https://github.com/unitreerobotics/unitree_ros2) (Sim2Real).

The code is **robot-agnostic**. Nothing in `src/`, `mujoco/mujoco_sim.py`,
the launch files or the offline tools contains a joint name, link name, joint
limit, leg length, gain or motor mode of a particular robot. Every robot is
described only by files named after it inside this package:

| File | Required | Content |
|---|---|---|
| `urdf/<robot>.urdf` or `urdf/<robot>.xacro` | yes | kinematics, inertials, joint limits / efforts / velocities |
| `config/<robot>_gait.yaml` | yes | CHAMP `gait.*` (nominal_height, swing_height, stance_duration, max velocities, cmd_vel acceleration slew, …) |
| `config/<robot>_joints.yaml` | yes | CHAMP `joints_map.{left_front,right_front,left_hind,right_hind}` (hip, upper, lower joint names) |
| `config/<robot>_links.yaml` | yes | CHAMP `links_map.*` leg chains + `links_map.base` + `links_map.imu` |
| `config/<robot>_sim.yaml` | MuJoCo | `sim.*` simulator tuning (actuator gains, joint damping, friction, spawn clearance, velocity limit; foot geom/site names for hand-written models) |
| `config/<robot>_lowcmd.yaml` | Sim2Real | `unitree_ros2_bridge` parameters: `kp`, `kd`, `motor_mode`, `contact_force_threshold`, `ramp_sec`, topics |
| `config/<robot>_body_pose.yaml` | optional | `body_pose_controller_node` parameters: IMU body roll/pitch loop gains (`kp`, `ki`, `kd`), `max_correction`, `max_rate`, `desired_rate`, `max_roll` / `max_pitch`, `max_error`, filter, rate, topics |
| `mujoco/<robot>.xml` | MuJoCo | MJCF model (generated from the URDF or hand-written) |
| `mujoco/assets/<robot>/*.obj` | MuJoCo | URDF visual meshes converted to OBJ (one per material, generated) |
| `rviz/<robot>_gait.rviz` | optional | RViz layout for the RViz launch |

Every launch file takes `robot:=<name>`; the name is the stem of the file in
`urdf/`. The package currently ships `go2` (Unitree Go2, official
`go2_description`), `b2` (Unitree B2, official `unitree_ros` `b2_description`)
and `xgo` (XGO SolidWorks export, no LowCmd).

```bash
python3 tools/champ_robot_files.py list      # robots found in urdf/
```

## Pipeline

1. **cmd_vel → joints**: `quadruped_controller_node`
   - `/cmd_vel` is clamped to the gait yaml maxima and ramped at
     `gait.max_linear_acceleration` / `max_angular_acceleration` (0 = step)
   - Gait planning (`LegController::velocityCommand`): trot, LF/RH against
     RF/LH, `stance_duration` of stance then a 0.25 s swing per leg (the two
     need not be equal; a longer stance adds a four-leg support window)
   - Start and stop never step a foot: the stride clock starts where RF/LH
     leave stance (every foot on the ground, the first swing rises from
     z = 0), and the node lets the velocity reach exactly zero — which resets
     CHAMP's gait and plants all feet at once — only on a tick where a foot has
     just touched down (or all four are in stance), never with a foot mid-swing
   - Inverse kinematics (`Kinematics::inverse`)
   - Publishes `/joint_states`, `/foot_contacts`

2. **joints → cmd_vel**: `state_estimation_node`
   - Forward kinematics (`QuadrupedLeg::foot_from_base`)
   - Velocity estimation (`Odometry::getVelocities`)
   - Publishes `/odom/raw`, `/cmd_vel/estimated`, `/foot`

Both nodes read the robot from the `urdf` parameter (the URDF text) plus the
three CHAMP yamls. `champ::URDF::getPose` sums the joint `origin xyz` along the
`links_map` chains, so every leg joint must have `rpy="0 0 0"`
(`tools/verify_urdf_champ_contract.py` checks this).

3. **MuJoCo**: `mujoco/mujoco_sim.py` loads `mujoco/<robot>.xml`, reads the
   joint order from `joints_map`, the base and IMU bodies from `links_map`, the
   joint velocity limits from the URDF and the tuning from `sim.*`.
   `tools/generate_mjcf.py` builds that model from the URDF: collision
   primitives (group 3, hidden by default) for the physics and the URDF
   `<visual>` meshes for the display, so the viewer shows the same robot as
   RViz. MuJoCo cannot read COLLADA, so `tools/champ_mesh_assets.py` converts
   each `.dae`/`.stl` into `mujoco/assets/<robot>/<mesh>_<n>.obj` (one file per
   material, coloured from the URDF/COLLADA materials); press `3` in the viewer
   to overlay the collision primitives. Robots whose URDF has only mesh
   collision keep a hand-written MJCF (`python3 tools/generate_mjcf.py <robot>
   --patch-visuals` inserts the visual meshes without replacing the collision).

4. **Sim2Real**: `unitree_ros2_bridge` maps CHAMP joints (LF, RF, LH, RH) to the
   Unitree motor order (FR, FL, RR, RL × hip/thigh/calf) with `joints_map`,
   clamps every target to the URDF `<limit lower upper>`, publishes
   `unitree_go/LowCmd` on `/lowcmd` (CRC computed over the wire layout, as in
   the unitree_ros2 `get_crc()`) and republishes `unitree_go/LowState` from
   `/lowstate` as `/joint_states`, `/foot_contacts`
   (`foot_force > contact_force_threshold`) and `/imu/data`
   (`frame_id = links_map.imu`). Before the first command it asks the robot's
   motion_switcher (`unitree_api/Request` on `/api/motion_switcher/request`)
   to `ReleaseMode`, like the unitree_ros2 `go2_stand_example` /
   `b2_stand_example`. The robot is just another node of the ROS 2 graph
   (CycloneDDS, domain 0); nothing from `unitree_sdk2` is used.

5. **IMU body roll/pitch** (optional, `body_pose_controller_node`): CHAMP's
   `BodyController` already rotates the stance feet opposite to the requested
   body roll/pitch/yaw before IK, so `/body_pose` tilts the body open loop. This
   node closes roll and pitch on the IMU, ~100 Hz, between the user and CHAMP:

   ```
   /body_pose (desired)  +  IMU orientation  ->  e = desired - measured
   command = desired + kp*e + ki*integral(e) - kd*body_rate      (bounded, slew-limited)
   /body_pose/corrected  ->  quadruped_controller_node body_pose  ->  gait + IK + PD as before
   ```

   The integral term is what cancels a slope (a ground tilt of θ needs a
   command of −θ); `max_correction` bounds it, `max_roll` / `max_pitch` bound
   the total command (both at once are checked against the IK reach and the
   URDF joint limits, standing and walking at the gait yaml maximum
   velocities), an error above `max_error` (fallen / lying / carried) or an
   IMU older than `imu_timeout_sec` releases the correction. The pose handed
   to CHAMP never steps: the correction moves at most `max_rate`, also when it
   is released (IMU lost, not standing, `body_pose.enabled` switched off), and
   the user's desired roll/pitch/yaw is ramped at `desired_rate`. Position
   passes through as before; yaw is ramped but never taken from the IMU (it
   drifts; CHAMP steers yaw with `cmd_vel`). The IMU axes are taken from the
   URDF fixed joints between `links_map.base` and `links_map.imu`
   (`body_pose.imu_mount_rpy` overrides them for an external IMU); the
   ROS-free loop is `include/body_orientation_controller.h`.

   The loop runs all the time, standing and walking: with a desired pose of
   zero it holds the body level to gravity (a slope is compensated up to
   `max_correction`); with a desired pitch of −0.15 rad it holds the nose
   0.15 rad up, on the flat and on slopes, as long as
   `|desired| + |slope| <= max_pitch`. It is a kinematic posture loop through
   the stance feet, not a balance controller: it does not stop a fall, and
   CHAMP's foot placement is unchanged.

## Build

```bash
source /opt/ros/humble/setup.bash       # or your ROS 2 distro
colcon build --packages-select champ champ_msgs champ_base_gait_planning
source install/setup.bash
```

Hardware Sim2Real needs the `unitree_go` and `unitree_api` message packages
from unitree_ros2 (`cyclonedds_ws/src/unitree`) so `unitree_ros2_bridge` is
built; without them CMake prints a warning and skips the bridge. Either build
unitree_ros2's `cyclonedds_ws` and source its `install/setup.bash` before
`colcon build`, or copy/symlink `unitree_go` and `unitree_api` into this
workspace's `src/`. Required apt packages:
`ros-$ROS_DISTRO-rmw-cyclonedds-cpp`, `ros-$ROS_DISTRO-rosidl-generator-dds-idl`.
`rosdep` does not know the `unitree_go`/`unitree_api` keys; skip them
(`--skip-keys "unitree_go unitree_api"`) when the packages are not sourced.
MuJoCo needs the `mujoco` Python package.

## Run

RViz kinematic walk (no physics, no LowCmd):

```bash
ros2 launch champ_base_gait_planning rviz_gait_test.launch.py robot:=go2
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}" -r 10
ros2 topic echo /cmd_vel/estimated
```

MuJoCo (`headless:=true` for no viewer):

```bash
ros2 launch champ_base_gait_planning mujoco_sim.launch.py robot:=b2
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# or Xbox One S 1708 Bluetooth (cmd_vel + body pose): ros2 launch xbox_one_s_teleop teleop.launch.py robot:=b2
```

Keep teleop speed below `gait.max_linear_velocity_x` of the robot; CHAMP
clamps anything above it silently.

Both the MuJoCo and the Sim2Real launch start the IMU body roll/pitch loop
when `config/<robot>_body_pose.yaml` exists (`body_pose_control:=auto`, the
default; `true` requires the file, `false` sends `/body_pose` straight to
CHAMP). `imu_topic:=<topic>` selects the `sensor_msgs/Imu` (fused
orientation) it reads: the simulator's / bridge's `/imu/data` by default, or an
external driver such as `/dog_imu_raw` (any QoS). MuJoCo can stand the
robot on a slope to exercise it:

```bash
ros2 launch champ_base_gait_planning mujoco_sim.launch.py robot:=go2 headless:=true floor_pitch:=0.10 floor_roll:=0.05
ros2 topic echo /body_pose/measured_rpy    # filtered body roll, pitch (rad); z = IMU yaw
ros2 topic echo /body_pose/correction      # closed-loop share of the command (rad); -> minus the slope
ros2 topic echo /odom/ground_truth         # MuJoCo truth: body roll/pitch should sit at the desired values
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"   # keeps levelling while trotting
```

Desired body pose = `geometry_msgs/Pose` on `/body_pose`, orientation as a
quaternion of roll/pitch/yaw (ROS convention: +pitch = nose **down**, +roll =
right side down). `--once` is enough, the loop and CHAMP keep the last pose;
the change is ramped at `desired_rate`. Anything beyond `max_roll` /
`max_pitch` is clamped, and the correction shares that budget (on a slope of
θ only `max_pitch − |θ|` is left for a desired pitch).

```bash
# level (default)
ros2 topic pub --once /body_pose geometry_msgs/msg/Pose "{orientation: {w: 1.0}}"
# nose up 0.10 rad (5.7 deg): RPY (0, -0.10, 0)
ros2 topic pub --once /body_pose geometry_msgs/msg/Pose "{orientation: {y: -0.04998, w: 0.99875}}"
# nose up 0.20 rad (11.5 deg, Go2 max_pitch): RPY (0, -0.20, 0)
ros2 topic pub --once /body_pose geometry_msgs/msg/Pose "{orientation: {y: -0.09983, w: 0.99500}}"
# roll 0.15 rad right side down: RPY (0.15, 0, 0)
ros2 topic pub --once /body_pose geometry_msgs/msg/Pose "{orientation: {x: 0.07493, w: 0.99719}}"
# body 3 cm lower (position passes through, CHAMP clamps it)
ros2 topic pub --once /body_pose geometry_msgs/msg/Pose "{position: {z: -0.03}, orientation: {w: 1.0}}"
ros2 param set /body_pose_controller_node body_pose.enabled false   # release the correction (ramped), pass /body_pose through
ros2 param set /body_pose_controller_node body_pose.enabled true
```

Quaternion of a roll/pitch pair (yaw 0):
`python3 -c "import math;r,p=0.0,-0.262;print(dict(x=math.sin(r/2)*math.cos(p/2),y=math.cos(r/2)*math.sin(p/2),z=-math.sin(r/2)*math.sin(p/2),w=math.cos(r/2)*math.cos(p/2)))"`

### Body pose limits per robot

Shipped `config/<robot>_body_pose.yaml` limits (rad, total command =
desired + correction) and the kinematic ceilings that
`verify_body_pose_controller` measures for them at `gait.nominal_height`
through BodyController → gait → IK, against the URDF joint limits (Go2/B2:
calf joint) or the leg reach (XGO), both signs:

| Robot | `max_roll` / `max_pitch` (yaml) | `max_correction` | pitch ceiling standing | pitch ceiling walking straight at max vx | pitch ceiling at max vx + vy + wz | roll ceiling (any case) |
|---|---|---|---|---|---|---|
| Go2 (`nominal_height` 0.30) | 0.25 (14.3°) / **0.20 (11.5°)** | 0.15 (8.6°) | 0.47 (27°) | 0.33 (19°) | 0.26 (15°) | ≥ 0.70 (40°) |
| B2 (`nominal_height` 0.50) | 0.25 (14.3°) / **0.25 (14.3°)** | 0.15 (8.6°) | 0.58 (33°) | 0.44 (25°) | 0.37 (21°) | ≥ 0.59 (34°) |
| XGO, MuJoCo only (`nominal_height` 0.10, leg reach 0.146 m) | 0.10 (5.7°) / **0.05 (2.9°)** | 0.05 (2.9°) | 0.24 (14°) | 0.10 (6°) | gait alone at the reach | ≥ 0.40 (23°) |

The yaml value must hold for every velocity the gait yaml allows, so it
follows the walking ceiling with margin (Go2 pitch 0.20 keeps ≥ 0.06 rad to
the calf joint limit; B2 0.25 keeps ≥ 0.12 rad). Beyond the ceiling CHAMP's
IK returns NaN and the legs freeze for that tick; a joint beyond its URDF
limit is clamped by the bridge (the foot lands elsewhere). A commanded 15°
nose-up (0.262 rad) is **clamped to the yaml `max_pitch`**, not tracked as 15°:
Go2 holds 11.5°, B2 14.3°, XGO 2.9°. To actually track 15°, raise the yaml
value and re-run `tools/run_offline_checks.sh <robot>`: B2 accepts 0.30 (17°)
at every gait velocity; Go2's walking vx+vy+wz ceiling is 0.26 rad so the
check rejects `max_pitch >= 0.27` unless the gait yaml velocities are lowered
too (standing 0.47, walking straight 0.33). XGO cannot tilt that far while
walking. Roll and pitch at once, `position.z` offsets and swing height
all consume the same leg reach, so keep `position.z` at 0 when using the
large tilts. Re-run `bash tools/run_offline_checks.sh <robot>` after any
change of these values: the walking-with-tilt check fails when the new limit
leaves the reach or a joint limit.

Real Unitree robot (Sport / motion services **off**, NIC cabled to the robot):

```bash
ros2 launch champ_base_gait_planning unitree_sim2real.launch.py robot:=go2 network_interface:=eth0
```

`unitree_lowcmd.launch.py robot:=<name>` is a short alias. With
`network_interface:=<nic>` the launch exports
`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` and a `CYCLONEDDS_URI` pinned to that
NIC for every node it starts (the same two variables unitree_ros2 `setup.sh`
exports); leave it empty when `setup.sh` is already sourced. The launch never
sets `ROS_DOMAIN_ID`: the robot is on domain 0, so keep it 0 / unset. The
bridge first asks the motion_switcher to `ReleaseMode` and, like the
unitree_ros2 stand examples, publishes no LowCmd while the robot reports a
motion service still active (it retries every second). It gives up and
continues only after 6 CheckMode **timeouts** (robot not on the graph); a
non-timeout error is still an answer, so LowCmd stays off. Then it holds the
measured pose and ramps to the CHAMP targets over `ramp_sec` at
`publish_rate`. CHAMP plans at 200 Hz and the motors are served at 500 Hz:
the bridge interpolates linearly between the last two CHAMP targets
(`interpolate_commands`, default true; one 5 ms period of latency) instead of
repeating each target, so a stiff joint PD does not get a torque step every
5 ms. If CHAMP commands stop for `command_timeout_sec`, the last pose is held
with extra damping. Never mix this with the Sport API.

Quick checks on the robot network: `ros2 topic hz /lowstate` (~500 Hz) and
`ros2 topic echo /api/motion_switcher/response --once` after a `ReleaseMode`.

With the body pose loop on the real robot, the loop reports `IMU roll/pitch
loop active` only after the bridge is publishing `/lowcmd` and `/imu/data`
(or the external `imu_topic`) is flowing. First test standing on a wedge and
watch `/body_pose/correction` settle to minus the wedge angle without
hunting; then tune `kp` / `ki` / `max_correction` in
`config/<robot>_body_pose.yaml` (the shipped values are conservative).

### Sim2Real with the body pose loop: procedure and cautions

```bash
# 0. offline, before touching the robot (limits, reach, gains of the yaml you are about to use)
bash tools/run_offline_checks.sh go2            # or b2

# 1. robot on and lying (damping), NIC cabled; the bridge asks the motion_switcher to release Sport
ros2 topic hz /lowstate                          # ~500 Hz, otherwise fix the NIC / CycloneDDS first
ros2 topic echo /lowstate --once | grep -A4 imu_state   # quaternion valid, robot lying level

# 2. first run without the loop: plain CHAMP standing / walking as before
ros2 launch champ_base_gait_planning unitree_sim2real.launch.py robot:=go2 network_interface:=eth0 body_pose_control:=false

# 3. loop on, robot on flat ground, standing only
ros2 launch champ_base_gait_planning unitree_sim2real.launch.py robot:=go2 network_interface:=eth0
ros2 topic echo /body_pose/measured_rpy          # roll/pitch near 0 (rad)
ros2 topic echo /body_pose/correction            # near 0, no hunting
# 4. wedge / slope under the robot: correction -> minus the slope, body stays level
# 5. desired tilt while standing (ramped at desired_rate), then back to level
ros2 topic pub --once /body_pose geometry_msgs/msg/Pose "{orientation: {y: -0.04998, w: 0.99875}}"
ros2 topic pub --once /body_pose geometry_msgs/msg/Pose "{orientation: {w: 1.0}}"
# 6. walking slowly on the flat, then on the slope
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
# emergency: release the correction (ramped) without stopping CHAMP
ros2 param set /body_pose_controller_node body_pose.enabled false
```

Cautions:

* Sport / motion services must be off; the bridge refuses to publish
  `/lowcmd` while `CheckMode` reports one active. Never run the Sport API and
  this launch together.
* Same as plain CHAMP Sim2Real: hold the robot (or use a stand / strap) at
  the first stand-up, `ramp_sec` ramps from the lying pose to the CHAMP
  stance; the loop is already active then and, on a slope, already tilts the
  stance.
* The loop moves the legs through the stance feet, up to `max_roll` /
  `max_pitch` (Go2: 0.25 / 0.20 rad). Keep hands and cables clear of the
  legs when tilting; the body lowers on one side.
* It is a kinematic posture loop, not a balance controller. It will not
  catch a fall, and beyond `max_error` (0.35 rad, ~20°) it releases the
  correction. Slopes above `max_correction` (0.15 rad, 8.6°) are only
  partially compensated.
* `desired_rate` / `max_rate` (0.4–0.6 rad/s) bound how fast the pose can
  change; when both move in the same direction the total is their sum
  (Go2: 1.0 rad/s). Do not raise them on the real robot before watching the
  ramp in MuJoCo (`/body_pose/corrected`).
* Keep `/body_pose` `position.z` at 0 while using large tilts (same leg
  reach). Keep teleop speed below `gait.max_linear_velocity_x`. `/cmd_vel`
  is ramped at `gait.max_linear_acceleration` / `max_angular_acceleration`
  (B2 0.30 m/s² / 0.40 rad/s²) so a full stick is not a velocity step.
* If `body_pose_controller_node` dies, CHAMP keeps the last
  `/body_pose/corrected` (a frozen correction, not a jump). Restart the
  launch; or run `body_pose_control:=false` to go back to plain CHAMP.
* IMU: the loop needs a fused orientation (`/lowstate` `imu_state` is; a raw
  gyro/accel topic is rejected with a warning). An external IMU that is not
  in `links_map.imu` needs `body_pose.imu_mount_rpy`.
* Watch the joint-limit warning of the bridge (`joint target(s) outside ...
  URDF limits were clamped`): with the shipped limits it must not appear;
  if it does, lower `max_pitch` / `max_roll` or the speed.
* Start with the shipped gains. If the body oscillates at 1–3 Hz while
  standing, lower `kp` and `ki` (halve them) before anything else; if a slope
  is compensated too slowly, raise `ki`, not `kp`.

## Offline checks (no ROS runtime)

```bash
bash tools/run_offline_checks.sh            # every robot in urdf/
bash tools/run_offline_checks.sh go2 b2     # selected robots
bash tools/run_offline_checks.sh --no-mujoco b2
```

Per robot this runs:

| Tool | Checks |
|---|---|
| `verify_urdf_champ_contract.py --robot R` | `links_map`/`joints_map` chains exist in the URDF, leg joints have `rpy=0`, hip axis `+X`, thigh/calf axes `+Y`, base/imu links exist, limits present, `nominal_height` inside the leg reach |
| `verify_champ_robot <urdf> <gait> <joints> <links>` (C++) | CHAMP IK/FK round trip, gait at yaml limits, odometry sign/scale, leg symmetry |
| `measure_gait_continuity <urdf> <gait> <joints> <links>` (C++) | first-swing lift, C1 stance↔swing foot velocity, signed vy+r·wz must not reset the phase clock, node stop gate vs hard-zero slam, +vx stance feet push −X |
| `validate_mujoco_against_urdf.py --robot R` | MJCF bodies/joints/inertials/limits/actuators vs URDF |
| `verify_mujoco_physics.py --robot R` | standing FK matches CHAMP, robot settles at `nominal_height`, feet in contact, IMU body |
| `verify_mujoco_walk.py --robot R [vx]` | replayed CHAMP trot moves forward (>= 55 % of commanded) |
| `verify_unitree_lowcmd <urdf> <joints>` (C++) | CHAMP↔Unitree motor index map, URDF limit clamp, LowCmd wire layout (812 B) and CRC32 (only robots with `_lowcmd.yaml`) |
| `verify_body_pose_controller <urdf> <gait> <joints> <links> <body_pose>` (C++) | quaternion/RPY conventions (incl. a real Unitree IMU sample), URDF IMU mount, CHAMP body-pose sign through BodyController→IK→FK, `max_roll`+`max_pitch` (all signs) reachable by IK and inside the URDF joint limits standing and walking at the gait yaml max velocities (prints the kinematic tilt ceilings), closed loop with the robot's gains: slope, trot wobble, set-point, saturation without wind-up, dead-band, IMU loss, not standing, clamp, unreachable desired tracked at the yaml limit, NaN IMU sample holds the last correction, ramped release when disabled, desired-pose ramp (only robots with `_body_pose.yaml`) |

`colcon test --packages-select champ_base_gait_planning` registers the same
checks as ctest, one set per robot found in `urdf/`.

`tools/dump_champ_gait <urdf> <gait> <joints> <links> [vx vy wz ticks swing stance]`
prints the CHAMP standing pose or a joint trajectory; the Python tools compile
it on demand (`g++`, `pkg-config tinyxml2 yaml-cpp`).

## Adding a Unitree robot

No code changes. For a robot `<name>`:

1. `urdf/<name>.urdf` — the official `<name>_description` URDF (meshes under
   `meshes/`). Leg joints in hip → thigh → calf → foot order per leg, all with
   `rpy="0 0 0"`, and `<limit lower upper effort velocity>` on every leg joint.
2. `config/<name>_joints.yaml` — `joints_map.left_front: [FL_hip_joint, FL_thigh_joint, FL_calf_joint]` etc.
3. `config/<name>_links.yaml` — `links_map.left_front: [FL_hip, FL_thigh, FL_calf, FL_foot]` etc.,
   `links_map.base: <root link>`, `links_map.imu: <imu link>`.
4. `config/<name>_gait.yaml` — start from `go2_gait.yaml` / `b2_gait.yaml` and scale
   `nominal_height`, `swing_height`, `stance_duration`, `max_linear_velocity_*`.
5. `config/<name>_lowcmd.yaml` — `kp`, `kd`, `motor_mode` and
   `contact_force_threshold` from the unitree_ros2 `<name>_stand_example`.
   Optional `config/<name>_body_pose.yaml` (start from `go2_body_pose.yaml`;
   `max_roll` / `max_pitch` must fit the leg reach at `nominal_height`, which
   `verify_body_pose_controller` checks) enables the IMU body roll/pitch loop.
6. `config/<name>_sim.yaml` + `python3 tools/generate_mjcf.py <name>` → `mujoco/<name>.xml`
   plus `mujoco/assets/<name>/*.obj` converted from the URDF visual meshes
   (URDF feet must be sphere collisions; otherwise hand-write the MJCF, list
   `sim.foot_geom_names` / `sim.foot_site_names` as `xgo_sim.yaml` does, then
   `python3 tools/generate_mjcf.py <name> --patch-visuals` to attach the STLs).
7. Optional `rviz/<name>_gait.rviz`.
8. `bash tools/run_offline_checks.sh <name>`.

## Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/cmd_vel` | `geometry_msgs/Twist` | in |
| `/cmd_vel/applied` | `geometry_msgs/Twist` | out (controller, after clamp + slew; compare with `/cmd_vel`) |
| `/body_pose` | `geometry_msgs/Pose` | in (desired body pose: position offset, roll/pitch/yaw) |
| `/body_pose/corrected` | `geometry_msgs/Pose` | out (`body_pose_controller_node`) → in (CHAMP `body_pose`, remapped by the launch) |
| `/body_pose/measured_rpy`, `/body_pose/correction` | `geometry_msgs/Vector3Stamped` | out (`body_pose_controller_node` diagnostics) |
| `/joint_states` | `sensor_msgs/JointState` | out (controller, or bridge/sim measured) |
| `/foot_contacts` | `champ_msgs/ContactsStamped` | out |
| `/imu/data` | `sensor_msgs/Imu` | out (bridge / sim) |
| `/cmd_vel/estimated` | `geometry_msgs/Twist` | out (estimator) |
| `/odom/raw` | `nav_msgs/Odometry` | out (estimator) |
| `/foot` | `visualization_msgs/MarkerArray` | out (estimator) |
| `/lowcmd` | `unitree_go/LowCmd` (DDS `rt/lowcmd`) | out (`unitree_ros2_bridge`, hardware) |
| `/lowstate` | `unitree_go/LowState` (DDS `rt/lowstate`) | in (`unitree_ros2_bridge`, hardware) |
| `/api/motion_switcher/request` | `unitree_api/Request` | out (`unitree_ros2_bridge`, ReleaseMode) |
| `/api/motion_switcher/response` | `unitree_api/Response` | in (`unitree_ros2_bridge`) |
