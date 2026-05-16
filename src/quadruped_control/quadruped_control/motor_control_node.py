#!/usr/bin/env python3
"""
motor_control_node.py
---------------------
Subscribes:
  /joint_angles  [std_msgs/Float32MultiArray]   12 floats in degrees

Writes to Teensy via USB Serial:
  Protocol: $ANGLES,<seq>,d1,d2,...,d12\n   (ASCII, ~170 bytes/line)

Reads from Teensy:
  $ACK,<seq>,1\n           — Jetson→Teensy ACK
  $HB,<teensy_seq>\n       — Teensy→Jetson heartbeat (10 Hz)

Publishes:
  /comm_stats  [std_msgs/String]  JSON drop stats every 1 s  (→ rosbag)
  /teensy_ack  [std_msgs/String]  raw ACK lines              (→ rosbag)
  /teensy_hb   [std_msgs/String]  raw HB  lines              (→ rosbag)
"""

import json, threading, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
import serial as pyserial


class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate',   115200)
        port = self.get_parameter('serial_port').value
        baud = int(self.get_parameter('baud_rate').value)

        # ── serial ──
        self._ser = pyserial.Serial(port, baud, timeout=0.005)
        self._lower_latency(port)

        # ── drop tracking ──
        self._tx_seq  = 0
        self._tx_sent = self._tx_acked = self._tx_nacked = 0

        self._hb_last     = None
        self._hb_received = self._hb_dropped = 0

        # ── ROS ──
        self.create_subscription(Float32MultiArray, '/joint_angles',
                                 self._angles_cb, 10)
        self._stats_pub = self.create_publisher(String, '/comm_stats', 10)
        self._ack_pub   = self.create_publisher(String, '/teensy_ack', 10)
        self._hb_pub    = self.create_publisher(String, '/teensy_hb',  10)

        threading.Thread(target=self._reader, daemon=True).start()
        self.create_timer(1.0, self._publish_stats)
        self.get_logger().info(f"MotorControlNode → {port} @ {baud}")

    # ── helpers ───────────────────────────────────────────────────────────────
    def _lower_latency(self, port: str):
        try:
            dev = port.split('/')[-1]
            with open(f"/sys/bus/usb-serial/devices/{dev}/latency_timer", 'w') as f:
                f.write('1')
            self.get_logger().info(f"USB latency timer → 1 ms ({dev})")
        except Exception as e:
            self.get_logger().warn(f"latency_timer: {e}")

    # ── TX ────────────────────────────────────────────────────────────────────
    def _angles_cb(self, msg: Float32MultiArray):
        if len(msg.data) != 12:
            self.get_logger().warn(f"Expected 12 angles, got {len(msg.data)}")
            return

        self._tx_seq += 1
        seq = self._tx_seq
        vals = ",".join(f"{v:.3f}" for v in msg.data)
        line = f"$ANGLES,{seq},{vals}\n"
        try:
            self._ser.write(line.encode())
            self._tx_sent += 1
        except Exception as e:
            self.get_logger().error(f"Serial write: {e}")

    # ── RX ────────────────────────────────────────────────────────────────────
    def _reader(self):
        buf = b""
        while True:
            try:
                chunk = self._ser.read(self._ser.in_waiting or 1)
                if chunk:
                    buf += chunk
                    while b'\n' in buf:
                        raw, buf = buf.split(b'\n', 1)
                        self._handle(raw.decode('ascii', errors='ignore').strip())
            except Exception as e:
                self.get_logger().error(f"Serial read: {e}")
                time.sleep(0.01)

    def _handle(self, line: str):
        if line.startswith('$ACK'):
            pub_msg = String(); pub_msg.data = line
            self._ack_pub.publish(pub_msg)
            try:
                _, seq_s, ok_s = line.split(',')
                if int(ok_s):
                    self._tx_acked  += 1
                else:
                    self._tx_nacked += 1
            except Exception:
                pass

        elif line.startswith('$HB'):
            pub_msg = String(); pub_msg.data = line
            self._hb_pub.publish(pub_msg)
            try:
                teensy_seq = int(line.split(',')[1])
                self._hb_received += 1
                if self._hb_last is not None:
                    gap = teensy_seq - self._hb_last - 1
                    if gap > 0:
                        self._hb_dropped += gap
                self._hb_last = teensy_seq
            except Exception:
                pass

    # ── stats ─────────────────────────────────────────────────────────────────
    def _publish_stats(self):
        tx_drop = (100.0 * (self._tx_sent - self._tx_acked) / self._tx_sent
                   if self._tx_sent else 0.0)
        rx_total = self._hb_received + self._hb_dropped
        rx_drop  = (100.0 * self._hb_dropped / rx_total if rx_total else 0.0)

        d = {
            "tx_sent":      self._tx_sent,
            "tx_acked":     self._tx_acked,
            "tx_drop_pct":  round(tx_drop, 2),
            "rx_hb_recv":   self._hb_received,
            "rx_hb_drop":   self._hb_dropped,
            "rx_drop_pct":  round(rx_drop, 2),
        }
        msg = String(); msg.data = json.dumps(d)
        self._stats_pub.publish(msg)
        self.get_logger().info(
            f"TX drop {tx_drop:.1f}%  RX drop {rx_drop:.1f}%  "
            f"(sent={self._tx_sent} acked={self._tx_acked})")


def main():
    rclpy.init()
    node = MotorControlNode()
    try:    rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node()

if __name__ == '__main__':
    main()