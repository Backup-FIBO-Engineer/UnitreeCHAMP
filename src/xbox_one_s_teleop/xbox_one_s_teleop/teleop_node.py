"""Xbox One S (1708) Bluetooth teleop.

Two exclusive modes (LB toggles):
  locomotion — left stick vx/vy, right stick X yaw, RB dead-man → /cmd_vel
  body pose  — right stick X/Y roll/pitch (rates) → /body_pose
A (green) publishes identity /body_pose (level). B zeros /cmd_vel.
"""
from __future__ import annotations

from typing import List, Optional

import rclpy
from geometry_msgs.msg import Pose, Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Joy
from std_msgs.msg import String

from xbox_one_s_teleop.mapping import (
    button_at,
    integrate_body_pose,
    locomotion_twist,
    quaternion_from_rpy,
    rising_edge,
)

MODE_LOCOMOTION = 'locomotion'
MODE_BODY_POSE = 'body_pose'


class XboxOneSTeleop(Node):
    def __init__(self) -> None:
        super().__init__('xbox_one_s_teleop')
        self._declare()

        qos = QoSProfile(depth=10)
        self.cmd_pub = self.create_publisher(Twist, self.get_parameter('cmd_vel_topic').value, qos)
        self.pose_pub = self.create_publisher(Pose, self.get_parameter('body_pose_topic').value, qos)
        self.mode_pub = self.create_publisher(String, 'xbox_teleop/mode', qos)
        self.create_subscription(Joy, self.get_parameter('joy_topic').value, self._on_joy, qos)

        self.mode = MODE_LOCOMOTION
        self.roll = 0.0
        self.pitch = 0.0
        self.prev_buttons: List[int] = []
        self._buttons_latched = False
        self.last_joy: Optional[Joy] = None
        self.last_joy_time = self.get_clock().now()

        rate = float(self.get_parameter('publish_rate').value)
        self.dt = 1.0 / max(rate, 1.0)
        self.create_timer(self.dt, self._on_timer)

        self.get_logger().info(
            'Xbox One S teleop: LB = locomotion ↔ body pose, RB = walk dead-man, '
            'A = reset body level, B = stop. Start in locomotion.'
        )
        self._publish_mode()

    def _declare(self) -> None:
        # Topics
        self.declare_parameter('joy_topic', 'joy')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('body_pose_topic', 'body_pose')
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('joy_timeout_sec', 0.5)
        self.declare_parameter('deadzone', 0.15)
        # SDL2 axes (ROS 2 joy_node). Linux js order is the other yaml.
        self.declare_parameter('axis_vx', 1)       # left stick Y
        self.declare_parameter('axis_vy', 0)       # left stick X
        self.declare_parameter('axis_yaw', 2)      # right stick X
        self.declare_parameter('axis_roll', 2)
        self.declare_parameter('axis_pitch', 3)    # right stick Y
        self.declare_parameter('invert_vx', True)    # stick up (−Y) → +vx (forward)
        self.declare_parameter('invert_vy', True)    # stick right → −vy (robot right)
        self.declare_parameter('invert_yaw', True)   # stick right → −yaw (turn right)
        self.declare_parameter('invert_roll', False)  # stick right → +roll (right side down)
        self.declare_parameter('invert_pitch', False)  # stick up (−Y) → nose up (−pitch)
        self.declare_parameter('button_mode', 4)      # LB
        self.declare_parameter('button_deadman', 5)   # RB
        self.declare_parameter('button_reset', 0)     # A
        self.declare_parameter('button_stop', 1)      # B
        # Limits: launch overlays gait / body_pose yaml when robot:= is set
        self.declare_parameter('max_linear_x', 0.5)
        self.declare_parameter('max_linear_y', 0.15)
        self.declare_parameter('max_angular_z', 0.6)
        self.declare_parameter('max_roll', 0.25)
        self.declare_parameter('max_pitch', 0.20)
        self.declare_parameter('roll_rate', 0.4)
        self.declare_parameter('pitch_rate', 0.4)

    def _p(self, name: str):
        return self.get_parameter(name).value

    def _on_joy(self, msg: Joy) -> None:
        buttons = list(msg.buttons)
        # First packet only latches state: a button already held at connect
        # must not look like a rising edge (would flip mode / reset pose).
        if not self._buttons_latched:
            self.prev_buttons = buttons
            self._buttons_latched = True
            self.last_joy = msg
            self.last_joy_time = self.get_clock().now()
            return
        if rising_edge(buttons, self.prev_buttons, int(self._p('button_mode'))):
            self.mode = (
                MODE_BODY_POSE if self.mode == MODE_LOCOMOTION else MODE_LOCOMOTION
            )
            self.get_logger().info(f'Xbox teleop mode: {self.mode}')
            self._publish_mode()
        if rising_edge(buttons, self.prev_buttons, int(self._p('button_reset'))):
            self.roll = 0.0
            self.pitch = 0.0
            self._publish_pose()
            self.get_logger().info('Body pose reset to level')
        if rising_edge(buttons, self.prev_buttons, int(self._p('button_stop'))):
            self.cmd_pub.publish(Twist())
            self.get_logger().info('cmd_vel zeroed')
        self.prev_buttons = buttons
        self.last_joy = msg
        self.last_joy_time = self.get_clock().now()

    def _on_timer(self) -> None:
        timeout = float(self._p('joy_timeout_sec'))
        age = (self.get_clock().now() - self.last_joy_time).nanoseconds * 1e-9
        if self.last_joy is None or age > timeout:
            self.cmd_pub.publish(Twist())
            return

        axes = list(self.last_joy.axes)
        buttons = list(self.last_joy.buttons)
        dz = float(self._p('deadzone'))

        if self.mode == MODE_LOCOMOTION:
            if button_at(buttons, int(self._p('button_deadman'))):
                vx, vy, wz = locomotion_twist(
                    axes,
                    vx_axis=int(self._p('axis_vx')),
                    vy_axis=int(self._p('axis_vy')),
                    yaw_axis=int(self._p('axis_yaw')),
                    invert_vx=bool(self._p('invert_vx')),
                    invert_vy=bool(self._p('invert_vy')),
                    invert_yaw=bool(self._p('invert_yaw')),
                    deadzone=dz,
                    max_linear_x=float(self._p('max_linear_x')),
                    max_linear_y=float(self._p('max_linear_y')),
                    max_angular_z=float(self._p('max_angular_z')),
                )
                twist = Twist()
                twist.linear.x = vx
                twist.linear.y = vy
                twist.angular.z = wz
                self.cmd_pub.publish(twist)
            else:
                self.cmd_pub.publish(Twist())
            return

        self.cmd_pub.publish(Twist())
        self.roll, self.pitch, moved = integrate_body_pose(
            self.roll,
            self.pitch,
            axes,
            roll_axis=int(self._p('axis_roll')),
            pitch_axis=int(self._p('axis_pitch')),
            invert_roll=bool(self._p('invert_roll')),
            invert_pitch=bool(self._p('invert_pitch')),
            deadzone=dz,
            roll_rate=float(self._p('roll_rate')),
            pitch_rate=float(self._p('pitch_rate')),
            max_roll=float(self._p('max_roll')),
            max_pitch=float(self._p('max_pitch')),
            dt=self.dt,
        )
        if moved:
            self._publish_pose()

    def _publish_pose(self) -> None:
        x, y, z, w = quaternion_from_rpy(self.roll, self.pitch, 0.0)
        pose = Pose()
        pose.orientation.x = x
        pose.orientation.y = y
        pose.orientation.z = z
        pose.orientation.w = w
        self.pose_pub.publish(pose)

    def _publish_mode(self) -> None:
        msg = String()
        msg.data = self.mode
        self.mode_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = XboxOneSTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.cmd_pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
