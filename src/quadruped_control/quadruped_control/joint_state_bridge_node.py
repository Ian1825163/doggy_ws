#!/usr/bin/env python3
"""
joint_state_bridge_node.py
--------------------------
Subscribes:
  /joint_angles  [std_msgs/Float32MultiArray]   12 real motor angles in degrees

Publishes:
  /joint_states  [sensor_msgs/JointState]        16 URDF joints in radians

The third motor drives the calf through a nonlinear linkage. The URDF keeps both
the real motor joint and the passive knee joint; this node computes the passive
knee angle at runtime for visualization, TF, logging, and later RL observations.
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

from quadruped_control.ik_solver import LinkageKneeMap
from quadruped_control.joint_utils import (
    COMMAND_LEG_ORDER,
    KNEE_JOINT_BY_LEG,
    MOTOR_JOINTS_BY_LEG,
)


class JointStateBridgeNode(Node):
    def __init__(self):
        super().__init__('joint_state_bridge_node')

        self.declare_parameter('input_topic', '/joint_angles')
        self.declare_parameter('joint_signs', [1.0] * 12)
        self.declare_parameter('joint_offsets_deg', [0.0] * 12)
        self.declare_parameter('motor3_linkage_signs', [1.0] * 4)
        self.declare_parameter('motor3_linkage_offsets_deg', [0.0] * 4)
        self.declare_parameter('knee_signs', [1.0] * 4)
        self.declare_parameter('knee_offsets_deg', [0.0] * 4)

        input_topic = self.get_parameter('input_topic').value
        self._joint_signs = self._array_param('joint_signs', 12)
        self._joint_offsets = self._deg_array_param('joint_offsets_deg', 12)
        self._motor3_linkage_signs = self._array_param('motor3_linkage_signs', 4)
        self._motor3_linkage_offsets = self._deg_array_param(
            'motor3_linkage_offsets_deg', 4
        )
        self._knee_signs = self._array_param('knee_signs', 4)
        self._knee_offsets = self._deg_array_param('knee_offsets_deg', 4)

        self._knee_maps = {leg: LinkageKneeMap() for leg in COMMAND_LEG_ORDER}

        self._pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_subscription(Float32MultiArray, input_topic, self._angles_cb, 10)

        self.get_logger().info(
            f"JointStateBridgeNode ready: {input_topic} -> /joint_states"
        )

    def _array_param(self, name: str, expected_len: int):
        values = [float(v) for v in self.get_parameter(name).value]
        if len(values) != expected_len:
            raise ValueError(f"{name} must have {expected_len} values")
        return values

    def _deg_array_param(self, name: str, expected_len: int):
        return [math.radians(v) for v in self._array_param(name, expected_len)]

    def _angles_cb(self, msg: Float32MultiArray):
        if len(msg.data) != 12:
            self.get_logger().warn(f"Expected 12 motor angles, got {len(msg.data)}")
            return

        motor_rad = [
            self._joint_signs[i] * math.radians(float(msg.data[i]))
            + self._joint_offsets[i]
            for i in range(12)
        ]

        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()

        for leg_i, leg in enumerate(COMMAND_LEG_ORDER):
            base = leg_i * 3
            roll, pitch, motor3 = motor_rad[base:base + 3]

            gamma = (
                self._motor3_linkage_signs[leg_i] * motor3
                + self._motor3_linkage_offsets[leg_i]
            )
            knee_raw = self._knee_maps[leg].compute(gamma)
            knee = self._knee_signs[leg_i] * knee_raw + self._knee_offsets[leg_i]

            out.name.extend(MOTOR_JOINTS_BY_LEG[leg])
            out.position.extend([roll, pitch, motor3])
            out.name.append(KNEE_JOINT_BY_LEG[leg])
            out.position.append(knee)

        self._pub.publish(out)


def main():
    rclpy.init()
    node = JointStateBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
