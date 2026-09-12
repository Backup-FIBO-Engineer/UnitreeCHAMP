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
(`urdf/<robot>.urdf`, `config/<robot>_{gait,joints,links,sim,lowcmd}.yaml`,
`mujoco/<robot>.xml`, `rviz/<robot>_gait.rviz`); the nodes, launch files and
tools read joint names, limits, leg geometry, gains and motor mode from them.
Shipped robots: `go2`, `b2` (both with LowCmd) and `xgo` (sim/RViz only).
See `src/champ_base_gait_planning/README.md` for the file contract and how to
add another Unitree robot without touching code.

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
and (robots with `_lowcmd.yaml`) the Unitree motor map, URDF limit clamp and CRC.
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
