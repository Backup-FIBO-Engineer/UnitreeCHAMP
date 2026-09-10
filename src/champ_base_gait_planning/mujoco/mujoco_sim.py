#!/usr/bin/env python3
"""MuJoCo physics simulation for XGO driven by CHAMP gait planning.

Topic convention used by the reviewed launch file:
  Subscribe: /joint_commands       sensor_msgs/JointState (desired positions)
  Publish:   /joint_states         sensor_msgs/JointState (simulated measurements)
  Publish:   /foot_contacts/sim    champ_msgs/ContactsStamped
  Publish:   /odom/ground_truth    nav_msgs/Odometry
  Publish:   /imu/data             sensor_msgs/Imu

The implementation resolves joint/actuator addresses from names instead of
assuming hard-coded qpos/qvel slices. It also converts the free-joint linear
velocity from world coordinates into base_link coordinates using the full
quaternion, not yaw only.
"""

from __future__ import annotations

import math
import os
import signal
import threading
import time
from typing import Dict, List, Optional

import mujoco
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from champ_msgs.msg import ContactsStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState

JOINT_NAMES = [
    'lf_hip_joint', 'lf_upper_leg_joint', 'lf_lower_leg_joint',
    'rf_hip_joint', 'rf_upper_leg_joint', 'rf_lower_leg_joint',
    'lh_hip_joint', 'lh_upper_leg_joint', 'lh_lower_leg_joint',
    'rh_hip_joint', 'rh_upper_leg_joint', 'rh_lower_leg_joint',
]

FOOT_GEOM_NAMES = ['lf_foot', 'rf_foot', 'lh_foot', 'rh_foot']


def _version_tuple(version: str) -> tuple[int, int, int]:
    values: List[int] = []
    for token in version.split('.')[:3]:
        digits = ''.join(ch for ch in token if ch.isdigit())
        values.append(int(digits) if digits else 0)
    while len(values) < 3:
        values.append(0)
    return tuple(values)  # type: ignore[return-value]


def _world_to_body_vector(quat_wxyz: np.ndarray, vector_world: np.ndarray) -> np.ndarray:
    """Rotate a world-frame vector into the body frame."""
    qw, qx, qy, qz = quat_wxyz
    norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if norm <= 1e-12:
        return vector_world.copy()
    qw, qx, qy, qz = (quat_wxyz / norm).tolist()

    rotation_world_from_body = np.array([
        [1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qz * qw),
         2.0 * (qx * qz + qy * qw)],
        [2.0 * (qx * qy + qz * qw), 1.0 - 2.0 * (qx * qx + qz * qz),
         2.0 * (qy * qz - qx * qw)],
        [2.0 * (qx * qz - qy * qw), 2.0 * (qy * qz + qx * qw),
         1.0 - 2.0 * (qx * qx + qy * qy)],
    ])
    return rotation_world_from_body.T @ vector_world


class MujocoSim(Node):
    def __init__(self) -> None:
        super().__init__('mujoco_sim')

        self.declare_parameter('headless', False)
        self.declare_parameter('xml_path', '')
        self.declare_parameter('joint_names', JOINT_NAMES)
        self.declare_parameter('foot_geom_names', FOOT_GEOM_NAMES)
        self.declare_parameter('imu_body_name', 'imu_link')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('command_topic', 'joint_commands')
        self.declare_parameter('joint_state_topic', 'joint_states')
        self.declare_parameter('contact_topic', 'foot_contacts/sim')
        self.declare_parameter('odom_topic', 'odom/ground_truth')
        self.declare_parameter('imu_topic', 'imu/data')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('realtime_factor', 1.0)
        self.declare_parameter('command_timeout_sec', 0.5)
        self.declare_parameter('max_joint_velocity', 1.5)

        if _version_tuple(mujoco.__version__) < (3, 1, 0):
            raise RuntimeError(
                f'MuJoCo {mujoco.__version__} is too old. position actuators use kv; '
                'install MuJoCo >= 3.1.0.'
            )

        joint_names = list(self.get_parameter('joint_names').value)
        foot_geom_names = list(self.get_parameter('foot_geom_names').value)
        if len(joint_names) != 12:
            raise ValueError('joint_names must contain 12 hinge names')
        if len(foot_geom_names) != 4:
            raise ValueError('foot_geom_names must contain 4 foot geoms (LF RF LH RH)')
        self.joint_names = joint_names
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.imu_body_name = str(self.get_parameter('imu_body_name').value)
        imu_body_name = self.imu_body_name

        configured_xml = str(self.get_parameter('xml_path').value).strip()
        if configured_xml:
            xml_path = os.path.abspath(os.path.expanduser(configured_xml))
        else:
            xml_path = os.path.join(
                get_package_share_directory('champ_base_gait_planning'),
                'mujoco', 'xgo.xml'
            )
        if not os.path.isfile(xml_path):
            raise FileNotFoundError(f'MuJoCo XML not found: {xml_path}')

        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        self.lock = threading.RLock()
        self.stop_event = threading.Event()

        self.joint_ids = np.array([
            self._require_id(mujoco.mjtObj.mjOBJ_JOINT, name) for name in self.joint_names
        ], dtype=np.int32)
        self.qpos_addresses = self.model.jnt_qposadr[self.joint_ids].copy()
        self.dof_addresses = self.model.jnt_dofadr[self.joint_ids].copy()
        self.actuator_ids = np.array([
            self._require_id(mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in self.joint_names
        ], dtype=np.int32)

        self.root_joint_id = self._require_id(mujoco.mjtObj.mjOBJ_JOINT, 'root')
        self.root_qpos_address = int(self.model.jnt_qposadr[self.root_joint_id])
        self.root_dof_address = int(self.model.jnt_dofadr[self.root_joint_id])

        # Offset of the IMU body from the floating base, used for lever-arm terms.
        imu_body_id = int(mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, imu_body_name))
        if imu_body_id >= 0:
            self.imu_lever_arm = self.model.body_pos[imu_body_id].astype(np.float64).copy()
        else:
            self.imu_lever_arm = np.zeros(3, dtype=np.float64)

        self.floor_geom_id = self._require_id(mujoco.mjtObj.mjOBJ_GEOM, 'floor')
        self.foot_geom_ids = np.array([
            self._require_id(mujoco.mjtObj.mjOBJ_GEOM, name) for name in foot_geom_names
        ], dtype=np.int32)
        self.foot_geom_to_index: Dict[int, int] = {
            int(geom_id): index for index, geom_id in enumerate(self.foot_geom_ids)
        }

        self.joint_ranges = self.model.jnt_range[self.joint_ids].copy()
        self.joint_limited = self.model.jnt_limited[self.joint_ids].astype(bool).copy()

        self.target_command: Optional[np.ndarray] = None
        self.control_command = np.zeros(len(self.joint_names), dtype=np.float64)
        self.last_command_monotonic = 0.0
        self.started = False

        command_topic = str(self.get_parameter('command_topic').value)
        joint_state_topic = str(self.get_parameter('joint_state_topic').value)
        contact_topic = str(self.get_parameter('contact_topic').value)
        odom_topic = str(self.get_parameter('odom_topic').value)
        imu_topic = str(self.get_parameter('imu_topic').value)

        self.command_sub = self.create_subscription(
            JointState, command_topic, self.command_callback, 10
        )
        self.joint_pub = self.create_publisher(JointState, joint_state_topic, 10)
        self.contact_pub = self.create_publisher(ContactsStamped, contact_topic, 10)
        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self.imu_pub = self.create_publisher(Imu, imu_topic, 10)

        publish_rate = float(self.get_parameter('publish_rate').value)
        if not math.isfinite(publish_rate) or publish_rate <= 0.0:
            raise ValueError('publish_rate must be > 0')
        self.publish_timer = self.create_timer(1.0 / publish_rate, self.publish_state)

        self.sim_thread = threading.Thread(target=self.simulation_loop, daemon=True)
        self.sim_thread.start()

        total_mass = float(np.sum(self.model.body_mass))
        self.get_logger().info(
            f'MuJoCo sim ready: xml={xml_path}, nq={self.model.nq}, nv={self.model.nv}, '
            f'nu={self.model.nu}, total_mass={total_mass:.9f} kg'
        )

    def _require_id(self, object_type: mujoco.mjtObj, name: str) -> int:
        object_id = int(mujoco.mj_name2id(self.model, object_type, name))
        if object_id < 0:
            raise RuntimeError(f'MuJoCo object not found: type={object_type}, name={name}')
        return object_id

    def command_callback(self, msg: JointState) -> None:
        if not msg.name or not msg.position:
            return

        name_to_index = {name: index for index, name in enumerate(msg.name)}
        try:
            command = np.array(
                [msg.position[name_to_index[name]] for name in self.joint_names],
                dtype=np.float64,
            )
        except (KeyError, IndexError, TypeError, ValueError):
            self.get_logger().warn('Rejected JointState: missing joint name or position')
            return

        if not np.all(np.isfinite(command)):
            self.get_logger().warn('Rejected JointState: command contains NaN or Inf')
            return

        command = self._clip_to_joint_ranges(command)

        with self.lock:
            if not self.started:
                # Put the model directly into CHAMP's first valid stance before enabling gravity.
                self.data.qpos[self.qpos_addresses] = command
                self.data.qvel[self.dof_addresses] = 0.0
                self.control_command[:] = command
                self.data.ctrl[self.actuator_ids] = self.control_command
                mujoco.mj_forward(self.model, self.data)
                self.started = True

            self.target_command = command
            self.last_command_monotonic = time.monotonic()

    def _clip_to_joint_ranges(self, command: np.ndarray) -> np.ndarray:
        clipped = command.copy()
        for index, limited in enumerate(self.joint_limited):
            if limited:
                clipped[index] = np.clip(
                    clipped[index], self.joint_ranges[index, 0], self.joint_ranges[index, 1]
                )
        return clipped

    def _update_control_for_step(self, dt: float, now_monotonic: float) -> None:
        if self.target_command is None:
            return

        timeout = float(self.get_parameter('command_timeout_sec').value)
        if timeout > 0.0 and now_monotonic - self.last_command_monotonic > timeout:
            # Stop advancing a stale gait and hold the current measured pose.
            target = self.data.qpos[self.qpos_addresses].copy()
        else:
            target = self.target_command

        max_velocity = float(self.get_parameter('max_joint_velocity').value)
        if max_velocity > 0.0 and math.isfinite(max_velocity):
            max_delta = max_velocity * dt
            error = target - self.control_command
            self.control_command += np.clip(error, -max_delta, max_delta)
        else:
            self.control_command[:] = target

        self.control_command[:] = self._clip_to_joint_ranges(self.control_command)
        self.data.ctrl[self.actuator_ids] = self.control_command

    def simulation_loop(self) -> None:
        realtime_factor = float(self.get_parameter('realtime_factor').value)
        if not math.isfinite(realtime_factor) or realtime_factor <= 0.0:
            self.get_logger().warn('Invalid realtime_factor; using 1.0')
            realtime_factor = 1.0

        physics_dt = float(self.model.opt.timestep)
        steps_per_chunk = max(1, int(round(0.005 / physics_dt)))
        next_deadline = time.monotonic()

        try:
            while rclpy.ok() and not self.stop_event.is_set():
                with self.lock:
                    if not self.started:
                        should_step = False
                    else:
                        should_step = True
                        for _ in range(steps_per_chunk):
                            self._update_control_for_step(physics_dt, time.monotonic())
                            mujoco.mj_step(self.model, self.data)

                if not should_step:
                    time.sleep(0.02)
                    next_deadline = time.monotonic()
                    continue

                next_deadline += physics_dt * steps_per_chunk / realtime_factor
                sleep_duration = next_deadline - time.monotonic()
                if sleep_duration > 0.0:
                    time.sleep(sleep_duration)
                else:
                    next_deadline = time.monotonic()
        except Exception as exc:
            self.get_logger().error(f'MuJoCo simulation loop failed: {exc}')

    def _read_contacts_locked(self) -> List[bool]:
        contacts = [False, False, False, False]
        for index in range(int(self.data.ncon)):
            contact = self.data.contact[index]
            geom1 = int(contact.geom1)
            geom2 = int(contact.geom2)
            if geom1 == self.floor_geom_id and geom2 in self.foot_geom_to_index:
                contacts[self.foot_geom_to_index[geom2]] = True
            elif geom2 == self.floor_geom_id and geom1 in self.foot_geom_to_index:
                contacts[self.foot_geom_to_index[geom1]] = True
        return contacts

    def publish_state(self) -> None:
        with self.lock:
            if not self.started:
                return

            joint_position = self.data.qpos[self.qpos_addresses].copy()
            joint_velocity = self.data.qvel[self.dof_addresses].copy()

            qpos_address = self.root_qpos_address
            dof_address = self.root_dof_address
            position_world = self.data.qpos[qpos_address:qpos_address + 3].copy()
            quaternion_wxyz = self.data.qpos[qpos_address + 3:qpos_address + 7].copy()
            linear_velocity_world = self.data.qvel[dof_address:dof_address + 3].copy()
            angular_velocity_body = self.data.qvel[dof_address + 3:dof_address + 6].copy()
            linear_acceleration_world = self.data.qacc[dof_address:dof_address + 3].copy()
            angular_acceleration_body = self.data.qacc[dof_address + 3:dof_address + 6].copy()
            foot_contacts = self._read_contacts_locked()

        linear_velocity_body = _world_to_body_vector(
            quaternion_wxyz, linear_velocity_world
        )
        # IMU reports specific force. At rest this is approximately +9.81 m/s^2 on body Z.
        # The sensor sits at imu_link, not at the free-joint origin, so add the rigid-body
        # lever-arm terms; they dominate during footfall impacts (verified against a
        # finite-difference of the imu_link world trajectory).
        gravity_world = np.array([0.0, 0.0, -9.81], dtype=np.float64)
        specific_force_body = _world_to_body_vector(
            quaternion_wxyz, linear_acceleration_world - gravity_world
        )
        specific_force_body = specific_force_body + np.cross(
            angular_acceleration_body, self.imu_lever_arm
        ) + np.cross(
            angular_velocity_body, np.cross(angular_velocity_body, self.imu_lever_arm)
        )
        now = self.get_clock().now().to_msg()

        joint_msg = JointState()
        joint_msg.header.stamp = now
        joint_msg.name = self.joint_names
        joint_msg.position = joint_position.tolist()
        joint_msg.velocity = joint_velocity.tolist()
        self.joint_pub.publish(joint_msg)

        contact_msg = ContactsStamped()
        contact_msg.header.stamp = now
        contact_msg.header.frame_id = self.base_frame
        contact_msg.contacts = foot_contacts
        self.contact_pub.publish(contact_msg)

        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = 'world'
        odom_msg.child_frame_id = self.base_frame
        odom_msg.pose.pose.position.x = float(position_world[0])
        odom_msg.pose.pose.position.y = float(position_world[1])
        odom_msg.pose.pose.position.z = float(position_world[2])
        odom_msg.pose.pose.orientation.w = float(quaternion_wxyz[0])
        odom_msg.pose.pose.orientation.x = float(quaternion_wxyz[1])
        odom_msg.pose.pose.orientation.y = float(quaternion_wxyz[2])
        odom_msg.pose.pose.orientation.z = float(quaternion_wxyz[3])
        odom_msg.twist.twist.linear.x = float(linear_velocity_body[0])
        odom_msg.twist.twist.linear.y = float(linear_velocity_body[1])
        odom_msg.twist.twist.linear.z = float(linear_velocity_body[2])
        odom_msg.twist.twist.angular.x = float(angular_velocity_body[0])
        odom_msg.twist.twist.angular.y = float(angular_velocity_body[1])
        odom_msg.twist.twist.angular.z = float(angular_velocity_body[2])
        self.odom_pub.publish(odom_msg)

        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = self.imu_body_name
        imu_msg.orientation.w = float(quaternion_wxyz[0])
        imu_msg.orientation.x = float(quaternion_wxyz[1])
        imu_msg.orientation.y = float(quaternion_wxyz[2])
        imu_msg.orientation.z = float(quaternion_wxyz[3])
        imu_msg.angular_velocity.x = float(angular_velocity_body[0])
        imu_msg.angular_velocity.y = float(angular_velocity_body[1])
        imu_msg.angular_velocity.z = float(angular_velocity_body[2])
        imu_msg.linear_acceleration.x = float(specific_force_body[0])
        imu_msg.linear_acceleration.y = float(specific_force_body[1])
        imu_msg.linear_acceleration.z = float(specific_force_body[2])
        imu_msg.orientation_covariance[0] = 1e-4
        imu_msg.orientation_covariance[4] = 1e-4
        imu_msg.orientation_covariance[8] = 2e-4
        imu_msg.angular_velocity_covariance[0] = 1e-4
        imu_msg.angular_velocity_covariance[4] = 1e-4
        imu_msg.angular_velocity_covariance[8] = 1e-4
        imu_msg.linear_acceleration_covariance[0] = 1e-3
        imu_msg.linear_acceleration_covariance[4] = 1e-3
        imu_msg.linear_acceleration_covariance[8] = 1e-3
        self.imu_pub.publish(imu_msg)

    def destroy_node(self) -> bool:
        self.stop_event.set()
        if self.sim_thread.is_alive():
            self.sim_thread.join(timeout=2.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[MujocoSim] = None
    try:
        node = MujocoSim()
        stop = node.stop_event

        def _request_stop(*_args) -> None:
            stop.set()

        signal.signal(signal.SIGINT, _request_stop)
        signal.signal(signal.SIGTERM, _request_stop)

        spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        spin_thread.start()

        headless = bool(node.get_parameter('headless').value)
        if headless:
            while rclpy.ok() and not stop.is_set():
                time.sleep(0.05)
        else:
            try:
                import mujoco.viewer
                with mujoco.viewer.launch_passive(node.model, node.data) as viewer:
                    while viewer.is_running() and rclpy.ok() and not stop.is_set():
                        with node.lock:
                            viewer.sync()
                        time.sleep(0.02)
            except KeyboardInterrupt:
                pass
            except Exception as exc:
                node.get_logger().warn(f'Viewer unavailable ({exc}); continuing headless')
                while rclpy.ok() and not stop.is_set():
                    time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.stop_event.set()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
