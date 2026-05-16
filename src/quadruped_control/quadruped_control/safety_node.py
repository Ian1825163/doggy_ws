#!/usr/bin/env python3
"""
safety_node.py
--------------
Gates candidate motor commands before they reach the Teensy bridge.

Subscribes:
  /joint_angles_unsafe  [std_msgs/Float32MultiArray]

Publishes:
  /joint_angles         [std_msgs/Float32MultiArray]
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from quadruped_control.joint_utils import (
    clamp_joint_angles_deg,
    stand_joint_angles_deg,
)


class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')

        self.declare_parameter('input_topic', '/joint_angles_unsafe')
        self.declare_parameter('output_topic', '/joint_angles')
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('timeout_s', 0.25)
        self.declare_parameter('angle_limit_deg', 70.0)
        self.declare_parameter('enabled', True)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        rate_hz = float(self.get_parameter('rate_hz').value)

        self._last_angles = None
        self._last_rx = None
        self._stand = stand_joint_angles_deg()

        self._pub = self.create_publisher(Float32MultiArray, output_topic, 10)
        self.create_subscription(Float32MultiArray, input_topic, self._cmd_cb, 10)
        self.create_timer(1.0 / rate_hz, self._tick)

        self.get_logger().info(
            f"SafetyNode ready: {input_topic} -> {output_topic} @ {rate_hz:.1f} Hz"
        )

    def _cmd_cb(self, msg: Float32MultiArray):
        if len(msg.data) != 12:
            self.get_logger().warn(f"Expected 12 angles, got {len(msg.data)}")
            return

        limit = float(self.get_parameter('angle_limit_deg').value)
        self._last_angles = clamp_joint_angles_deg(msg.data, limit)
        self._last_rx = self.get_clock().now()

    def _tick(self):
        enabled = bool(self.get_parameter('enabled').value)
        timeout_s = float(self.get_parameter('timeout_s').value)

        angles = self._stand
        if enabled and self._last_angles is not None and self._last_rx is not None:
            age_s = (self.get_clock().now() - self._last_rx).nanoseconds * 1e-9
            if age_s <= timeout_s:
                angles = self._last_angles

        msg = Float32MultiArray()
        msg.data = [float(v) for v in angles]
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
