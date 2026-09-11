# UnitreeUniversalCHAMP

CHAMP gait / IK / odometry, MuJoCo simulation and Unitree LowCmd Sim2Real for
Unitree quadrupeds (branch `UnitreeUniversalCHAMP`, merged from `Go2EDU` and
`B2EDU`). The workspace contains:

- `src/champ` — kinematics, leg controller, odometry (headers)
- `src/champ_msgs` — `ContactsStamped`
- `src/champ_base_gait_planning` — ROS 2 nodes, robot descriptions, MuJoCo, Sim2Real DDS bridge

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
source /opt/ros/jazzy/setup.bash
colcon build --packages-select champ champ_msgs champ_base_gait_planning
source install/setup.bash
```

Hardware Sim2Real also needs [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2)
(`--cmake-args -DCMAKE_PREFIX_PATH=/opt/unitree_robotics`) so `unitree_dds_bridge` is compiled.

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

The bridge publishes Unitree DDS `rt/lowcmd` (`LowCmd_`, CRC, motor order
FR/FL/RR/RL) with the `kp` / `kd` / `motor_mode` from
`config/<robot>_lowcmd.yaml`, clamps every joint target to the URDF limits of
the selected robot, and reads `rt/lowstate`. Stand with Sport first; the launch
releases Sport and holds the pose, then ramps into CHAMP over `ramp_sec`. Do not
mix with the Sport API.
