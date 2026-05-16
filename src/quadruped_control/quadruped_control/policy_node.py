#!/usr/bin/env python3
"""
policy_node.py
--------------
First deployment scaffold for an RL policy.

For now this node publishes a safe standing command unless `policy_enabled` is
set and a real inference backend is added. It keeps the runtime interface stable:

  /gait_cmd -> policy_node -> /joint_angles_unsafe

The safety node should gate `/joint_angles_unsafe` into `/joint_angles` before
anything reaches the motor serial bridge.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int8

from quadruped_control.joint_utils import stand_joint_angles_deg


class PolicyNode(Node):
    def __init__(self):
        super().__init__('policy_node')

        self.declare_parameter('output_topic', '/joint_angles_unsafe')
        self.declare_parameter('control_rate_hz', 50.0)
        self.declare_parameter('policy_enabled', False)
        self.declare_parameter('model_path', '')

        output_topic = self.get_parameter('output_topic').value
        rate_hz = float(self.get_parameter('control_rate_hz').value)

        self._gait = 0
        self._stand = stand_joint_angles_deg()
        self._warned_no_policy = False

        self._pub = self.create_publisher(Float32MultiArray, output_topic, 10)
        self.create_subscription(Int8, '/gait_cmd', self._gait_cb, 10)
        self.create_timer(1.0 / rate_hz, self._tick)

        self.get_logger().info(
            f"PolicyNode ready @ {rate_hz:.1f} Hz -> {output_topic}"
        )

    def _gait_cb(self, msg: Int8):
        self._gait = int(msg.data)

    def _tick(self):
        policy_enabled = bool(self.get_parameter('policy_enabled').value)
        model_path = str(self.get_parameter('model_path').value)

        if self._gait == 0 or not policy_enabled:
            angles = self._stand
        else:
            angles = self._infer_policy_or_stand(model_path)

        msg = Float32MultiArray()
        msg.data = [float(v) for v in angles]
        self._pub.publish(msg)

    def _infer_policy_or_stand(self, model_path: str):
        if not self._warned_no_policy:
            self.get_logger().warn(
                "policy_enabled is true, but inference is not wired yet; "
                f"holding stand pose. model_path='{model_path}'"
            )
            self._warned_no_policy = True
        return self._stand


def main():
    rclpy.init()
    node = PolicyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
