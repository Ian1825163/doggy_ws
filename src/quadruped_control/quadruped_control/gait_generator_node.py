#!/usr/bin/env python3
"""
gait_generator_node.py
----------------------
Subscribes:
  /gait_cmd  [std_msgs/Int8]     0=stand 1=walk 2=trot
  /imu       [sensor_msgs/Imu]   for body pose compensation

Publishes:
  /joint_angles  [std_msgs/Float32MultiArray]
      12 floats (degrees), order:
      LB1 LB2 LB3 | RB1 RB2 RB3 | RF1 RF2 RF3 | LF1 LF2 LF3
      (same sign convention as your Teensy sendMotorPacketDeg call)

Timer: 50 Hz (same CTRL_US=20 ms as Teensy was doing)

When RL is ready: replace the trot()/walk() block with
  angles = policy.infer(obs)  and publish directly.
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, Float32MultiArray
from sensor_msgs.msg import Imu
from scipy.spatial.transform import Rotation

from quadruped_control.ik_solver import Leg, _clamp, LIMIT_DEG

# ── gait parameters (mirror of Teensy) ───────────────────────────────────────
T        = 2.0
LIFT     = 10.0
DISP     = 30.0
X0       = 26.086
Y0       = 0.0
Z0       = -150.0
PHASE_RATE = 1.0 / T

def _wrap01(p):
    p = math.fmod(p, 1.0)
    return p + 1.0 if p < 0 else p

def _sinusoidal_trajectory(leg: Leg, p: float, duty: float):
    p = _wrap01(p)
    if p < duty:
        u = p / duty
        leg.set_target(X0, Y0 + DISP*(0.5 - u), Z0)
    else:
        u = (p - duty) / (1.0 - duty)
        leg.set_target(X0, Y0 + DISP*(u - 0.5), Z0 + LIFT*math.sin(math.pi*u))

def _standing(leg: Leg): leg.set_target(X0, Y0, Z0)


class GaitGeneratorNode(Node):
    def __init__(self):
        super().__init__('gait_generator_node')

        # ── state ──
        self._gait   = 0      # 0=stand 1=walk 2=trot
        self._phase  = 0.0

        # IMU calibration
        self._calib_roll, self._calib_pitch = [], []
        self._roll_off = self._pitch_off = 0.0
        self._calibrated = False
        self._N = 50
        self._roll = self._pitch = 0.0

        # Leg instances (IK state lives here)
        self._LB = Leg()
        self._RB = Leg()
        self._RF = Leg()
        self._LF = Leg()

        # ── pubs / subs ──
        self._pub = self.create_publisher(Float32MultiArray, '/joint_angles', 10)
        self.create_subscription(Int8, '/gait_cmd', self._gait_cb, 10)
        self.create_subscription(Imu,  '/imu',      self._imu_cb,  10)

        # 50 Hz control timer
        DT_S = 0.02
        self._dt = DT_S
        self.create_timer(DT_S, self._control_tick)

        self.get_logger().info("GaitGeneratorNode ready @ 50 Hz")

    # ── callbacks ─────────────────────────────────────────────────────────────
    def _gait_cb(self, msg: Int8):
        self._gait = int(msg.data)
        self.get_logger().info(f"gait mode → {self._gait}")

    def _imu_cb(self, msg: Imu):
        q = msg.orientation
        rot = Rotation.from_quat([q.x, q.y, q.z, q.w])
        roll, pitch, _ = rot.as_euler('xyz')

        if not self._calibrated:
            self._calib_roll.append(roll)
            self._calib_pitch.append(pitch)
            if len(self._calib_roll) >= self._N:
                self._roll_off  = sum(self._calib_roll)  / self._N
                self._pitch_off = sum(self._calib_pitch) / self._N
                self._calibrated = True
                self.get_logger().info(
                    f"IMU calibrated  roll_off={self._roll_off:.4f}  "
                    f"pitch_off={self._pitch_off:.4f}")
            return
        self._roll  = roll  - self._roll_off
        self._pitch = pitch - self._pitch_off

    # ── 50 Hz tick ────────────────────────────────────────────────────────────
    def _control_tick(self):
        self._phase = _wrap01(self._phase + PHASE_RATE * self._dt)

        LB, RB, RF, LF = self._LB, self._RB, self._RF, self._LF

        if self._gait == 2:                         # TROT
            duty = 0.6
            _sinusoidal_trajectory(RB, self._phase, duty)
            _sinusoidal_trajectory(LF, self._phase, duty)
            _sinusoidal_trajectory(RF, _wrap01(self._phase + 0.5), duty)
            _sinusoidal_trajectory(LB, _wrap01(self._phase + 0.5), duty)

        elif self._gait == 1:                       # WALK
            duty = 0.75
            _sinusoidal_trajectory(RB, self._phase, duty)
            _sinusoidal_trajectory(RF, _wrap01(self._phase + 0.25), duty)
            _sinusoidal_trajectory(LB, _wrap01(self._phase + 0.50), duty)
            _sinusoidal_trajectory(LF, _wrap01(self._phase + 0.75), duty)

        else:                                       # STAND
            _standing(LB); _standing(RB)
            _standing(RF); _standing(LF)

        for leg in (LB, RB, RF, LF):
            leg.compute_ik()

        # ── sign convention: mirrors Teensy sendMotorPacketDeg call ──
        #   LB1  LB2  LB3  | RB1  RB2  RB3  | RF1  RF2  RF3  | LF1  LF2  LF3
        a = [
             LB.angles_deg[0], -LB.angles_deg[1],  LB.angles_deg[2],
            -RB.angles_deg[0],  RB.angles_deg[1], -RB.angles_deg[2],
             RF.angles_deg[0],  RF.angles_deg[1], -RF.angles_deg[2],
            -LF.angles_deg[0], -LF.angles_deg[1],  LF.angles_deg[2],
        ]

        msg = Float32MultiArray()
        msg.data = [float(_clamp(v, -LIMIT_DEG, LIMIT_DEG)) for v in a]
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = GaitGeneratorNode()
    try:    rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node()

if __name__ == '__main__':
    main()