#!/usr/bin/env python3
"""ROS <-> Isaac Sim relay (bridges the Python-version gap).

Isaac Sim's Python is 3.11 and cannot import ROS 2 Humble's rclpy (3.10). This
node runs in the ROS Python, and shuttles data to/from the Isaac process over a
localhost UDP/JSON link:

  * subscribes the per-arm command topics -> forwards latest to the sim (cmd_port)
  * receives joint states from the sim (state_port) -> publishes /joint_states

Run::  ros2 run openarm_teleop_vr openarm_isaac_relay
"""

import json
import socket
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class IsaacRelay(Node):
    def __init__(self):
        super().__init__("openarm_isaac_relay")
        self.declare_parameter("sim_host", "127.0.0.1")
        self.declare_parameter("cmd_port", 6001)     # -> sim
        self.declare_parameter("state_port", 6002)   # <- sim
        self.declare_parameter("forward_rate", 120.0)
        self.declare_parameter("left_cmd_topic",
                               "/left_forward_position_controller/commands")
        self.declare_parameter("right_cmd_topic",
                               "/right_forward_position_controller/commands")
        g = self.get_parameter
        self.sim_addr = (g("sim_host").value, int(g("cmd_port").value))

        self._lock = threading.Lock()
        self._left = None
        self._right = None
        self._tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rx.bind(("0.0.0.0", int(g("state_port").value)))

        self.create_subscription(Float64MultiArray, g("left_cmd_topic").value,
                                 self._left_cb, 10)
        self.create_subscription(Float64MultiArray, g("right_cmd_topic").value,
                                 self._right_cb, 10)
        self.js_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_timer(1.0 / float(g("forward_rate").value), self._forward)

        threading.Thread(target=self._state_loop, daemon=True).start()
        self.get_logger().info(
            f"Relay up: cmds -> {self.sim_addr}, states <- :{g('state_port').value}")

    def _left_cb(self, msg):
        with self._lock:
            self._left = list(msg.data)

    def _right_cb(self, msg):
        with self._lock:
            self._right = list(msg.data)

    def _forward(self):
        with self._lock:
            payload = json.dumps({"left": self._left, "right": self._right}).encode()
        try:
            self._tx.sendto(payload, self.sim_addr)
        except OSError:
            pass

    def _state_loop(self):
        while rclpy.ok():
            try:
                data, _ = self._rx.recvfrom(65536)
            except OSError:
                break
            try:
                msg = json.loads(data.decode())
            except (ValueError, UnicodeDecodeError):
                continue
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = list(msg.get("name", []))
            js.position = [float(p) for p in msg.get("position", [])]
            js.velocity = [float(v) for v in msg.get("velocity", [])]
            self.js_pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = IsaacRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
