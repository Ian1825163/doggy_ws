#!/usr/bin/env python3
"""
sts_feedback_test_node.py
-------------------------
Direct Feetech STS feedback reader for bring-up tests.

This node talks the STS/SCS packet protocol directly on a servo bus serial port
and reads the present-position block from each configured servo ID. It is meant
for a USB-to-servo-bus adapter, or for a Teensy firmware that forwards these
binary packets between USB Serial and the STS bus.

The current `src/teensy_motor.ino` writes binary STS packets on Serial4, but it
does not forward USB Serial packets to Serial4 or send feedback back to Jetson.
So this node will not work through that firmware until a bridge/readback path is
added on the Teensy side.
"""

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial as pyserial


INST_READ = 0x02
ADDR_PRESENT_POSITION = 56
READ_PRESENT_BLOCK_LEN = 8


def le_u16(data, offset):
    return int(data[offset]) | (int(data[offset + 1]) << 8)


def decode_load_raw(raw):
    value = raw & 0x03FF
    return -value if raw & 0x0400 else value


def build_packet(servo_id, instruction, params):
    body = [int(servo_id) & 0xFF, len(params) + 2, int(instruction) & 0xFF]
    body.extend(int(p) & 0xFF for p in params)
    checksum = (~sum(body)) & 0xFF
    return bytes([0xFF, 0xFF, *body, checksum])


def build_read_packet(servo_id, address, length):
    return build_packet(servo_id, INST_READ, [address, length])


class StsFeedbackTestNode(Node):
    def __init__(self):
        super().__init__('sts_feedback_test_node')

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 1000000)
        self.declare_parameter('servo_ids', list(range(1, 13)))
        self.declare_parameter('poll_hz', 5.0)
        self.declare_parameter('read_timeout_s', 0.03)
        self.declare_parameter('center_tick', 2048)
        self.declare_parameter('topic', '/sts_feedback_raw')

        port = self.get_parameter('serial_port').value
        baud = int(self.get_parameter('baud_rate').value)
        poll_hz = float(self.get_parameter('poll_hz').value)
        topic = self.get_parameter('topic').value

        self._ids = [int(v) for v in self.get_parameter('servo_ids').value]
        self._timeout_s = float(self.get_parameter('read_timeout_s').value)
        self._center_tick = int(self.get_parameter('center_tick').value)

        self._ser = pyserial.Serial(port, baud, timeout=self._timeout_s)
        self._pub = self.create_publisher(String, topic, 10)
        self.create_timer(1.0 / poll_hz, self._tick)

        self._last_log = 0.0
        self.get_logger().info(
            f"STS feedback test reading IDs {self._ids} on {port} @ {baud}, "
            f"publishing {topic}"
        )

    def _tick(self):
        samples = []
        errors = []

        for servo_id in self._ids:
            try:
                params = self._read_registers(
                    servo_id,
                    ADDR_PRESENT_POSITION,
                    READ_PRESENT_BLOCK_LEN,
                )
                samples.append(self._decode_present_block(servo_id, params))
            except Exception as exc:
                errors.append({'id': servo_id, 'error': str(exc)})

        msg = String()
        msg.data = json.dumps({
            'stamp_ns': self.get_clock().now().nanoseconds,
            'register_start': ADDR_PRESENT_POSITION,
            'register_len': READ_PRESENT_BLOCK_LEN,
            'samples': samples,
            'errors': errors,
        })
        self._pub.publish(msg)

        now = time.monotonic()
        if now - self._last_log >= 1.0:
            self._last_log = now
            if samples:
                first = samples[0]
                self.get_logger().info(
                    f"STS feedback ok={len(samples)} err={len(errors)} "
                    f"first id={first['id']} pos={first['position_deg']:.2f} deg"
                )
            else:
                self.get_logger().warn(f"STS feedback ok=0 err={len(errors)}")

    def _decode_present_block(self, servo_id, params):
        if len(params) < READ_PRESENT_BLOCK_LEN:
            raise ValueError(f"short present block: {len(params)} bytes")

        position_tick = le_u16(params, 0)
        speed_raw = le_u16(params, 2)
        load_raw = le_u16(params, 4)
        voltage_raw = int(params[6])
        temperature_c = int(params[7])

        return {
            'id': servo_id,
            'position_tick': position_tick,
            'position_deg': (position_tick - self._center_tick) * 360.0 / 4096.0,
            'speed_raw': speed_raw,
            'load_raw': load_raw,
            'load_signed_raw': decode_load_raw(load_raw),
            'voltage_raw': voltage_raw,
            'voltage_v_est': voltage_raw / 10.0,
            'temperature_c': temperature_c,
        }

    def _read_registers(self, servo_id, address, length):
        packet = build_read_packet(servo_id, address, length)
        self._ser.reset_input_buffer()
        self._ser.write(packet)
        status_id, status, params = self._read_status_packet()

        if status_id != servo_id:
            raise ValueError(f"expected ID {servo_id}, got {status_id}")
        if status != 0:
            raise ValueError(f"servo status error 0x{status:02X}")
        if len(params) != length:
            raise ValueError(f"expected {length} params, got {len(params)}")
        return params

    def _read_status_packet(self):
        deadline = time.monotonic() + self._timeout_s

        while time.monotonic() < deadline:
            b = self._read_exact(1, deadline)
            if b == b'\xFF':
                b2 = self._read_exact(1, deadline)
                if b2 == b'\xFF':
                    break
        else:
            raise TimeoutError('no status header')

        servo_id = self._read_exact(1, deadline)[0]
        length = self._read_exact(1, deadline)[0]
        if length < 2:
            raise ValueError(f"bad status length {length}")

        rest = self._read_exact(length, deadline)
        status = rest[0]
        params = rest[1:-1]
        checksum = rest[-1]

        expected = (~sum([servo_id, length, status, *params])) & 0xFF
        if checksum != expected:
            raise ValueError(
                f"checksum mismatch got 0x{checksum:02X}, expected 0x{expected:02X}"
            )

        return servo_id, status, list(params)

    def _read_exact(self, nbytes, deadline):
        out = bytearray()
        while len(out) < nbytes:
            if time.monotonic() >= deadline:
                raise TimeoutError(f"wanted {nbytes} bytes, got {len(out)}")
            chunk = self._ser.read(nbytes - len(out))
            if chunk:
                out.extend(chunk)
        return bytes(out)


def main():
    rclpy.init()
    node = StsFeedbackTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
