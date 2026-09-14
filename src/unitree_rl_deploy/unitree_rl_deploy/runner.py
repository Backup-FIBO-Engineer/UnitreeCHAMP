"""ROS 2 node: IMU + joints + /cmd_vel + base velocity → policy → joint_commands.

joint_commands is the topic MuJoCo and unitree_ros2_bridge consume.
Base linear velocity (training `lin_vel`) comes from nav_msgs/Odometry and,
optionally, DLS `dls2_interface/BaseState`, rotated into the body frame.
The controller then adds ω × r_com so the value matches Isaac `root_lin_vel_b`.
"""
from __future__ import annotations

import time
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32MultiArray

from unitree_rl_deploy.config import DeployConfig
from unitree_rl_deploy.controller import PolicyController, SensorSample
from unitree_rl_deploy.observation import (
    LIN_VEL_FRAME_WORLD,
    as_quat_xyzw,
    as_vector3,
    linear_velocity_in_body_frame,
    quat_xyzw_is_valid,
)
from unitree_rl_deploy.policy import load_policy


def _package_share() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory('unitree_rl_deploy'))
    except Exception:
        return Path(__file__).resolve().parents[1]


def _import_base_state_msg():
    try:
        from dls2_interface.msg import BaseState
    except ImportError:
        return None
    return BaseState


class PolicyRunner(Node):
    def __init__(self) -> None:
        super().__init__(
            'policy_runner',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )
        cfg = self._load_config()
        share = _package_share()
        search = [
            share / 'policies',
            Path(__file__).resolve().parents[1] / 'policies',
        ]
        policy_override = str(self._param('policy', cfg.policy_path) or '').strip()
        self.policy_kind = 'stand' if policy_override in ('', 'stand', 'none') else policy_override
        policy = load_policy(policy_override, cfg.num_obs, cfg.num_actions, search)
        self.controller = PolicyController(cfg, policy)
        self.cfg = cfg
        self._needs_lin_vel = cfg.observation.needs_lin_vel

        self._q: Optional[np.ndarray] = None
        self._dq: Optional[np.ndarray] = None
        self._quat = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
        self._ang_vel = np.zeros(3, dtype=np.float32)
        self._accel = np.zeros(3, dtype=np.float32)
        self._orientation_valid = False
        self._lin_vel: Optional[np.ndarray] = None
        self._cmd = np.zeros(3, dtype=np.float32)
        self._last_imu_mono = 0.0
        self._last_joint_mono = 0.0
        self._last_cmd_mono = 0.0
        self._last_lin_vel_mono = 0.0
        self._has_cmd = False
        self._started = False
        self._name_to_index = {name: i for i, name in enumerate(cfg.joint_names)}

        self.command_pub = self.create_publisher(JointState, cfg.command_topic, 10)
        self.obs_pub = self.create_publisher(Float32MultiArray, 'rl/observation', 10)
        self.action_pub = self.create_publisher(Float32MultiArray, 'rl/action', 10)

        self.create_subscription(JointState, cfg.joint_state_topic, self._on_joints, 10)
        self.create_subscription(Imu, cfg.imu_topic, self._on_imu, qos_profile_sensor_data)
        self.create_subscription(Twist, cfg.cmd_vel_topic, self._on_cmd, 10)
        if cfg.odom_topic:
            self.create_subscription(Odometry, cfg.odom_topic, self._on_odom, 10)
        if cfg.base_state_topic:
            base_state_cls = _import_base_state_msg()
            if base_state_cls is None:
                raise RuntimeError(
                    f'base_state_topic={cfg.base_state_topic!r} is set but '
                    'dls2_interface is not installed. Publish nav_msgs/Odometry instead '
                    'or install dls2_interface for /base_state.')
            self.create_subscription(
                base_state_cls, cfg.base_state_topic, self._on_base_state, 10)

        period = 1.0 / cfg.control_rate
        self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f'RL policy runner: {cfg.num_actions} joints, obs {cfg.num_obs} '
            f'({cfg.observation.history} x {cfg.observation.single_size}), '
            f'{cfg.control_rate:.0f} Hz, policy={self.policy_kind!r}, '
            f'filter_actions={cfg.use_filter_actions}'
        )
        if self._needs_lin_vel:
            sources = []
            if cfg.odom_topic:
                sources.append(f'odom={cfg.odom_topic} ({cfg.lin_vel_frame}-frame twist)')
            if cfg.base_state_topic:
                sources.append(f'base_state={cfg.base_state_topic} (world-frame linear vel)')
            self.get_logger().info('lin_vel from ' + ', '.join(sources))
        if self.policy_kind == 'stand':
            self.get_logger().warn(
                'No policy file: holding default_angles. Pass policy:=/path/to/policy.pt '
                'or set policy_path in the robot yaml.'
            )

    def _param(self, name: str, default=None):
        if self.has_parameter(name):
            value = self.get_parameter(name).value
            if value is not None:
                return value
        return default

    def _load_config(self) -> DeployConfig:
        raw = {}
        for param in self._parameters.values():
            raw[param.name] = param.value
        # Nested yaml arrives as dotted keys (observation.terms).
        nested: dict = {}
        for key, value in raw.items():
            cursor = nested
            parts = key.split('.')
            for part in parts[:-1]:
                cursor = cursor.setdefault(part, {})
            cursor[parts[-1]] = value
        # Launch may pass a yaml file through parameters; from_mapping accepts both.
        return DeployConfig.from_mapping(nested)

    def _on_cmd(self, msg: Twist) -> None:
        self._cmd = self.controller.clamp_command(msg.linear.x, msg.linear.y, msg.angular.z)
        self._has_cmd = True
        self._last_cmd_mono = time.monotonic()

    def _on_imu(self, msg: Imu) -> None:
        self._quat = np.array(
            [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w],
            dtype=np.float32,
        )
        self._ang_vel = np.array(
            [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
            dtype=np.float32,
        )
        self._accel = np.array(
            [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z],
            dtype=np.float32,
        )
        cov0 = float(msg.orientation_covariance[0]) if msg.orientation_covariance else 0.0
        finite = bool(np.all(np.isfinite(self._quat))) and float(np.linalg.norm(self._quat)) > 1e-6
        self._orientation_valid = finite and cov0 != -1.0
        self._last_imu_mono = time.monotonic()

    def _set_body_lin_vel(self, linear_xyz, source_frame: str, quat_xyzw=None) -> None:
        orientation = quat_xyzw if quat_xyzw is not None else self._quat
        if source_frame == LIN_VEL_FRAME_WORLD and not quat_xyzw_is_valid(orientation):
            if quat_xyzw_is_valid(self._quat):
                orientation = self._quat
            else:
                self.get_logger().warn(
                    'world-frame lin_vel arrived without a usable orientation; waiting',
                    throttle_duration_sec=2.0)
                return
        try:
            self._lin_vel = linear_velocity_in_body_frame(
                linear_xyz, source_frame, orientation)
        except ValueError as exc:
            self.get_logger().warn(f'lin_vel rejected: {exc}', throttle_duration_sec=2.0)
            return
        self._last_lin_vel_mono = time.monotonic()

    def _on_odom(self, msg: Odometry) -> None:
        linear = as_vector3(msg.twist.twist.linear)
        orientation = as_quat_xyzw(msg.pose.pose.orientation)
        self._set_body_lin_vel(linear, self.cfg.lin_vel_frame, orientation)

    def _on_base_state(self, msg) -> None:
        # DLS BaseState.velocity.linear is world-frame (same as deploy/run_controller_ros2.py).
        linear = as_vector3(msg.velocity.linear)
        orientation = as_quat_xyzw(msg.pose.orientation)
        self._set_body_lin_vel(linear, LIN_VEL_FRAME_WORLD, orientation)

    def _on_joints(self, msg: JointState) -> None:
        n = self.cfg.num_actions
        q = np.full(n, np.nan, dtype=np.float32)
        dq = np.zeros(n, dtype=np.float32)
        count = min(len(msg.name), len(msg.position))
        for i in range(count):
            index = self._name_to_index.get(msg.name[i])
            if index is None:
                continue
            q[index] = float(msg.position[i])
            if i < len(msg.velocity):
                dq[index] = float(msg.velocity[i])
        if not np.all(np.isfinite(q)):
            missing = [
                name for name, index in self._name_to_index.items()
                if not np.isfinite(q[index])
            ]
            self.get_logger().warn(
                f'joint_states missing {missing}; waiting', throttle_duration_sec=2.0)
            return
        self._q = q
        self._dq = dq
        self._last_joint_mono = time.monotonic()

    def _hold_or_stand(self) -> None:
        if self._started:
            return
        self._publish_targets(self.cfg.default_angles)

    def _on_timer(self) -> None:
        now = time.monotonic()
        if self._q is None:
            # MuJoCo does not step until a command arrives. Hold the default pose
            # so the simulator (and a robot still ramping) can start.
            self._publish_targets(self.cfg.default_angles)
            return
        imu_age = now - self._last_imu_mono
        joint_age = now - self._last_joint_mono
        if imu_age > self.cfg.imu_timeout_sec or joint_age > self.cfg.joint_timeout_sec:
            self.get_logger().warn(
                f'stale sensors imu={imu_age:.3f}s joints={joint_age:.3f}s; holding last pose',
                throttle_duration_sec=1.0)
            self._hold_or_stand()
            return
        if self._needs_lin_vel:
            lin_age = now - self._last_lin_vel_mono
            if self._lin_vel is None:
                self.get_logger().warn(
                    'waiting for lin_vel (odom/base_state); holding default pose',
                    throttle_duration_sec=2.0)
                self._hold_or_stand()
                return
            if (
                self.cfg.lin_vel_timeout_sec > 0.0
                and lin_age > self.cfg.lin_vel_timeout_sec
            ):
                self.get_logger().warn(
                    f'stale lin_vel age={lin_age:.3f}s; holding last pose',
                    throttle_duration_sec=1.0)
                self._hold_or_stand()
                return
        cmd = self._cmd
        if (
            self.cfg.cmd_timeout_sec > 0.0
            and (
                not self._has_cmd
                or (now - self._last_cmd_mono) > self.cfg.cmd_timeout_sec
            )
        ):
            cmd = np.zeros(3, dtype=np.float32)
        try:
            targets = self.controller.targets(SensorSample(
                q=self._q,
                dq=self._dq if self._dq is not None else np.zeros(self.cfg.num_actions),
                quat_xyzw=self._quat,
                ang_vel=self._ang_vel,
                cmd_vx=float(cmd[0]),
                cmd_vy=float(cmd[1]),
                cmd_wz=float(cmd[2]),
                lin_vel=None if not self._needs_lin_vel else self._lin_vel,
                accel=self._accel,
                orientation_valid=self._orientation_valid,
                time_sec=now,
            ))
        except (ValueError, RuntimeError) as exc:
            self.get_logger().error(f'policy tick failed: {exc}', throttle_duration_sec=1.0)
            return
        self._started = True
        self._publish_targets(targets)
        obs_msg = Float32MultiArray()
        obs_msg.data = self.controller.history.stacked.tolist()
        self.obs_pub.publish(obs_msg)
        act_msg = Float32MultiArray()
        act_msg.data = self.controller.last_action.tolist()
        self.action_pub.publish(act_msg)

    def _publish_targets(self, targets: np.ndarray) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.cfg.joint_names)
        msg.position = [float(v) for v in targets]
        self.command_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PolicyRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
