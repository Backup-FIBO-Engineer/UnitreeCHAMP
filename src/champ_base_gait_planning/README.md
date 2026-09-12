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
| `config/<robot>_gait.yaml` | yes | CHAMP `gait.*` (nominal_height, swing_height, stance_duration, max velocities, …) |
| `config/<robot>_joints.yaml` | yes | CHAMP `joints_map.{left_front,right_front,left_hind,right_hind}` (hip, upper, lower joint names) |
| `config/<robot>_links.yaml` | yes | CHAMP `links_map.*` leg chains + `links_map.base` + `links_map.imu` |
| `config/<robot>_sim.yaml` | MuJoCo | `sim.*` simulator tuning (actuator gains, joint damping, friction, spawn clearance, velocity limit; foot geom/site names for hand-written models) |
| `config/<robot>_lowcmd.yaml` | Sim2Real | `unitree_ros2_bridge` parameters: `kp`, `kd`, `motor_mode`, `contact_force_threshold`, `ramp_sec`, topics |
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
   - Gait planning (`LegController::velocityCommand`)
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
```

Keep teleop speed below `gait.max_linear_velocity_x` of the robot; CHAMP
clamps anything above it silently.

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
motion service still active (it retries every second; if the service never
answers it gives up after 6 attempts and continues). Then it holds the
measured pose and ramps to the CHAMP targets over `ramp_sec` at
`publish_rate`. If CHAMP commands stop for `command_timeout_sec`, the last
pose is held with extra damping. Never mix this with the Sport API.

Quick checks on the robot network: `ros2 topic hz /lowstate` (~500 Hz) and
`ros2 topic echo /api/motion_switcher/response --once` after a `ReleaseMode`.

## Offline checks (no ROS runtime)

```bash
bash tools/run_offline_checks.sh            # every robot in urdf/
bash tools/run_offline_checks.sh go2 b2     # selected robots
```

Per robot this runs:

| Tool | Checks |
|---|---|
| `verify_urdf_champ_contract.py --robot R` | `links_map`/`joints_map` chains exist in the URDF, leg joints have `rpy=0`, hip axis `+X`, thigh/calf axes `+Y`, base/imu links exist, limits present, `nominal_height` inside the leg reach |
| `verify_champ_robot <urdf> <gait> <joints> <links>` (C++) | CHAMP IK/FK round trip, gait at yaml limits, odometry sign/scale, leg symmetry |
| `validate_mujoco_against_urdf.py --robot R` | MJCF bodies/joints/inertials/limits/actuators vs URDF |
| `verify_mujoco_physics.py --robot R` | standing FK matches CHAMP, robot settles at `nominal_height`, feet in contact, IMU body |
| `verify_mujoco_walk.py --robot R [vx]` | replayed CHAMP trot moves forward (>= 55 % of commanded) |
| `verify_unitree_lowcmd <urdf> <joints>` (C++) | CHAMP↔Unitree motor index map, URDF limit clamp, LowCmd wire layout (812 B) and CRC32 (only robots with `_lowcmd.yaml`) |

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
