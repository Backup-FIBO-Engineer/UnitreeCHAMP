"""Xbox One S (1708) Bluetooth teleop.

Left stick vx/vy, right stick X yaw, RB dead-man → /cmd_vel.
B zeros /cmd_vel.
"""
from __future__ import annotations

from typing import List, Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Joy

from xbox_one_s_teleop.mapping import (
    button_at,
    locomotion_twist,
    rising_edge,
)


class XboxOneSTeleop(Node):
    def __init__(self) -> None:
        super().__init__('xbox_one_s_teleop')
        self._declare()

        qos = QoSProfile(depth=10)
        self.cmd_pub = self.create_publisher(Twist, self.get_parameter('cmd_vel_topic').value, qos)
        self.create_subscription(Joy, self.get_parameter('joy_topic').value, self._on_joy, qos)

        self.prev_buttons: List[int] = []
        self._buttons_latched = False
        self.last_joy: Optional[Joy] = None
        self.last_joy_time = self.get_clock().now()

        rate = float(self.get_parameter('publish_rate').value)
        self.dt = 1.0 / max(rate, 1.0)
        self.create_timer(self.dt, self._on_timer)

        self._last_cmd_log = self.get_clock().now()
        self.get_logger().info(
            'Xbox One S teleop: RB = walk dead-man, B = stop. '
            f'invert_vx={self._p("invert_vx")} invert_vy={self._p("invert_vy")} '
            f'invert_yaw={self._p("invert_yaw")} '
            '(stick-forward must publish /cmd_vel linear.x > 0; overlay invert_vx:=true/false).'
        )

    def _declare(self) -> None:
        self.declare_parameter('joy_topic', 'joy')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('joy_timeout_sec', 0.5)
        self.declare_parameter('deadzone', 0.15)
        # SDL2 axes (ROS 2 joy_node). Linux js order is the other yaml.
        self.declare_parameter('axis_vx', 1)       # left stick Y
        self.declare_parameter('axis_vy', 0)       # left stick X
        self.declare_parameter('axis_yaw', 2)      # right stick X
        self.declare_parameter('invert_vx', True)    # stick up (−Y) → +vx (forward)
        self.declare_parameter('invert_vy', True)    # stick right → −vy (robot right)
        self.declare_parameter('invert_yaw', True)   # stick right → −yaw (turn right)
        self.declare_parameter('button_deadman', 5)   # RB
        self.declare_parameter('button_stop', 1)      # B
        # Limits: launch overlays max_cmd from <robot>_rl.yaml when robot:= is set
        self.declare_parameter('max_linear_x', 0.5)
        self.declare_parameter('max_linear_y', 0.15)
        self.declare_parameter('max_angular_z', 0.6)

    def _p(self, name: str):
        return self.get_parameter(name).value

    def _on_joy(self, msg: Joy) -> None:
        buttons = list(msg.buttons)
        # First packet only latches state: a button already held at connect
        # must not look like a rising edge (would stop immediately).
        if not self._buttons_latched:
            self.prev_buttons = buttons
            self._buttons_latched = True
            self.last_joy = msg
            self.last_joy_time = self.get_clock().now()
            return
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
            now = self.get_clock().now()
            if (now - self._last_cmd_log).nanoseconds >= 1_000_000_000:
                self._last_cmd_log = now
                self.get_logger().info(
                    f'/cmd_vel vx={vx:.3f} vy={vy:.3f} wz={wz:.3f} '
                    '(+vx is forward; invert_vx:=true/false if this sign is wrong)'
                )
        else:
            self.cmd_pub.publish(Twist())


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
