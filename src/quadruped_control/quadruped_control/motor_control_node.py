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
from serial import SerialException


class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate',   115200)
        self._port = self.get_parameter('serial_port').value
        self._baud = int(self.get_parameter('baud_rate').value)

        # ── serial ──
        self._ser = None
        self._serial_lock = threading.Lock()
        self._last_serial_error_log = 0.0
        self._connect_serial()

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
        self.create_timer(1.0, self._reconnect_timer)
        self.create_timer(1.0, self._publish_stats)
        self.get_logger().info(
            f"MotorControlNode → {self._port} @ {self._baud}")

    # ── helpers ───────────────────────────────────────────────────────────────
    def _connect_serial(self):
        with self._serial_lock:
            if self._ser is not None and self._ser.is_open:
                return True

            try:
                ser = pyserial.Serial(
                    self._port, self._baud, timeout=0.005, write_timeout=0.02)
                self._ser = ser
            except Exception as e:
                now = time.monotonic()
                if now - self._last_serial_error_log >= 1.0:
                    self._last_serial_error_log = now
                    self.get_logger().warn(
                        f"Serial reconnect failed ({self._port}): {e}")
                return False

        self._lower_latency(self._port)
        self.get_logger().info(f"Serial connected: {self._port} @ {self._baud}")
        return True

    def _disconnect_serial(self, context: str, error: Exception):
        with self._serial_lock:
            ser = self._ser
            self._ser = None

        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass

        now = time.monotonic()
        if now - self._last_serial_error_log >= 1.0:
            self._last_serial_error_log = now
            self.get_logger().error(
                f"Serial {context}: {error}; disconnected, will retry")

    def _reconnect_timer(self):
        if self._ser is None:
            self._connect_serial()

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
            with self._serial_lock:
                if self._ser is None:
                    return
                self._ser.write(line.encode())
            self._tx_sent += 1
        except SerialException as e:
            self._disconnect_serial("write", e)
        except Exception as e:
            self._disconnect_serial("write", e)

    # ── RX ────────────────────────────────────────────────────────────────────
    def _reader(self):
        buf = b""
        while True:
            try:
                ser = self._ser
                if ser is None:
                    time.sleep(0.05)
                    continue

                chunk = ser.read(ser.in_waiting or 1)
                if chunk:
                    buf += chunk
                    while b'\n' in buf:
                        raw, buf = buf.split(b'\n', 1)
                        self._handle(raw.decode('ascii', errors='ignore').strip())
            except SerialException as e:
                self._disconnect_serial("read", e)
                buf = b""
                time.sleep(0.05)
            except Exception as e:
                self._disconnect_serial("read", e)
                buf = b""
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
