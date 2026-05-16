#!/usr/bin/env python3
"""
keyboard_node.py
----------------
Raw-mode keyboard → /gait_cmd [std_msgs/Int8]
  s = 0 STAND
  w = 1 WALK
  t = 2 TROT
  q = quit (sends STAND first)
"""
import sys, tty, termios, threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8

NAMES = {0: "STAND", 1: "WALK", 2: "TROT"}
KEYS  = {'s': 0, 'w': 1, 't': 2}

class KeyboardNode(Node):
    def __init__(self):
        super().__init__('keyboard_node')
        self.pub = self.create_publisher(Int8, '/gait_cmd', 10)
        self._send(0)
        self.get_logger().info("Keyboard: s=STAND  w=WALK  t=TROT  q=QUIT")
        threading.Thread(target=self._loop, daemon=True).start()

    def _send(self, mode: int):
        msg = Int8(); msg.data = mode
        self.pub.publish(msg)
        self.get_logger().info(f"gait → {NAMES.get(mode, mode)}")

    def _loop(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while rclpy.ok():
                k = sys.stdin.read(1)
                if k in KEYS:
                    self._send(KEYS[k])
                elif k == 'q':
                    self._send(0)
                    rclpy.shutdown(); break
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

def main():
    rclpy.init()
    node = KeyboardNode()
    try:    rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node()

if __name__ == '__main__':
    main()