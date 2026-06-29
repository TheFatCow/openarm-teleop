#!/usr/bin/env python3
"""ROS 2 adapter for the open OpenArm bimanual teleop core (MVP, relative mode).

Subscribes to the VR bridge's relative controller topics and ``/joint_states``,
runs :class:`OpenArmTeleopCore` at a fixed rate, applies per-joint step limiting
for safety, and publishes ``Float64MultiArray`` joint commands (7 arm joints + 1
gripper value, in metres) for each arm.

The output topics are backend-agnostic: the Isaac Sim node and (later) the real
OpenArm ``ros2_control`` forward-position controllers both consume them, so
switching sim <-> real never touches this node.
"""

import threading

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float64MultiArray

from openarm_teleop_vr.teleop_core import (
    ControllerInput, IKConfig, OpenArmTeleopCore, TeleopInputs,
)

LEFT_JOINTS = [f"openarm_left_joint{i}" for i in range(1, 8)]
RIGHT_JOINTS = [f"openarm_right_joint{i}" for i in range(1, 8)]
LEFT_FINGER = "openarm_left_finger_joint1"
RIGHT_FINGER = "openarm_right_finger_joint1"


class OpenArmTeleopVRNode(Node):
    def __init__(self):
        super().__init__("openarm_teleop_vr_node")
        self.cb_group = ReentrantCallbackGroup()
        self._declare_parameters()

        urdf_path = self.get_parameter("urdf_path").value
        if not urdf_path:
            raise ValueError("urdf_path parameter is required")

        self.control_rate = float(self.get_parameter("control_rate").value)
        self.grip_threshold = float(self.get_parameter("grip_threshold").value)

        max_step_deg = list(self.get_parameter("max_step_deg_per_joint").value)
        if len(max_step_deg) != 7:
            raise ValueError("max_step_deg_per_joint must have 7 entries")
        base = np.deg2rad(np.asarray(max_step_deg, dtype=np.float64))
        self.max_step_rad = np.concatenate([base, base])  # 14

        self.core = OpenArmTeleopCore(
            urdf_path,
            grip_threshold=self.grip_threshold,
            position_scale_xyz=list(self.get_parameter("position_scale_xyz").value),
            axis_matrix=np.asarray(self.get_parameter("axis_matrix").value,
                                   dtype=np.float64).reshape(3, 3),
            gripper_max=float(self.get_parameter("gripper_max").value),
            ik_cfg=IKConfig(),
        )
        self.get_logger().info("Open teleop core initialized")

        self._lock = threading.Lock()
        self._joint_positions = {}
        self._joint_states_received = False
        self.left_in = ControllerInput()
        self.right_in = ControllerInput()

        self._setup_io()
        self.create_timer(1.0 / self.control_rate, self._control_loop,
                          callback_group=self.cb_group)
        self.get_logger().info(f"Node ready - control rate {self.control_rate} Hz")

    def _declare_parameters(self):
        self.declare_parameter("urdf_path", "")
        self.declare_parameter("control_rate", 100.0)
        self.declare_parameter("grip_threshold", 0.5)
        self.declare_parameter("gripper_max", 0.044)
        self.declare_parameter("max_step_deg_per_joint", [4.0] * 7)
        self.declare_parameter("position_scale_xyz", [1.0, 1.0, 1.0])
        # controller-frame -> robot-base-frame mapping (row-major 3x3). Identity by
        # default; tune empirically once the VR stream is live (see README).
        self.declare_parameter("axis_matrix",
                               [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
        # input topics (match the bridge defaults)
        self.declare_parameter("left_pose_topic", "/vr_left_controller/pose")
        self.declare_parameter("right_pose_topic", "/vr_right_controller/pose")
        self.declare_parameter("left_grip_topic", "/vr_left_controller/grip")
        self.declare_parameter("right_grip_topic", "/vr_right_controller/grip")
        self.declare_parameter("left_trigger_topic", "/vr_left_controller/trigger")
        self.declare_parameter("right_trigger_topic", "/vr_right_controller/trigger")
        # output topics
        self.declare_parameter("left_cmd_topic",
                               "/left_forward_position_controller/commands")
        self.declare_parameter("right_cmd_topic",
                               "/right_forward_position_controller/commands")

    def _setup_io(self):
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        g = self.cb_group
        self.create_subscription(PoseStamped, p("left_pose_topic"),
                                 self._left_pose_cb, 10, callback_group=g)
        self.create_subscription(PoseStamped, p("right_pose_topic"),
                                 self._right_pose_cb, 10, callback_group=g)
        self.create_subscription(Float32, p("left_grip_topic"),
                                 self._left_grip_cb, 10, callback_group=g)
        self.create_subscription(Float32, p("right_grip_topic"),
                                 self._right_grip_cb, 10, callback_group=g)
        self.create_subscription(Float32, p("left_trigger_topic"),
                                 self._left_trigger_cb, 10, callback_group=g)
        self.create_subscription(Float32, p("right_trigger_topic"),
                                 self._right_trigger_cb, 10, callback_group=g)
        self.create_subscription(JointState, "/joint_states",
                                 self._joint_state_cb, 10, callback_group=g)
        self.left_pub = self.create_publisher(Float64MultiArray, p("left_cmd_topic"), 10)
        self.right_pub = self.create_publisher(Float64MultiArray, p("right_cmd_topic"), 10)

    # -- callbacks ---------------------------------------------------------
    @staticmethod
    def _pose_to(ci: ControllerInput, msg: PoseStamped):
        ci.position = np.array([msg.pose.position.x, msg.pose.position.y,
                                msg.pose.position.z], dtype=np.float64)
        ci.quat_xyzw = (msg.pose.orientation.x, msg.pose.orientation.y,
                        msg.pose.orientation.z, msg.pose.orientation.w)
        ci.valid = True

    def _left_pose_cb(self, msg):
        with self._lock:
            self._pose_to(self.left_in, msg)

    def _right_pose_cb(self, msg):
        with self._lock:
            self._pose_to(self.right_in, msg)

    def _left_grip_cb(self, msg):
        self.left_in.grip = float(msg.data)

    def _right_grip_cb(self, msg):
        self.right_in.grip = float(msg.data)

    def _left_trigger_cb(self, msg):
        self.left_in.trigger = float(msg.data)

    def _right_trigger_cb(self, msg):
        self.right_in.trigger = float(msg.data)

    def _joint_state_cb(self, msg: JointState):
        with self._lock:
            for name, pos in zip(msg.name, msg.position):
                self._joint_positions[name] = float(pos)
            self._joint_states_received = True

    # -- control loop ------------------------------------------------------
    def _current_q14(self):
        if not self._joint_states_received:
            return None
        vals = []
        for name in LEFT_JOINTS + RIGHT_JOINTS:
            v = self._joint_positions.get(name)
            if v is None:
                return None
            vals.append(v)
        return np.asarray(vals, dtype=np.float64)

    def _limit_step(self, target_q, current_q):
        delta = np.clip(target_q - current_q, -self.max_step_rad, self.max_step_rad)
        return current_q + delta

    def _control_loop(self):
        with self._lock:
            current_q = self._current_q14()
            inputs = TeleopInputs(
                left=ControllerInput(self.left_in.position, self.left_in.quat_xyzw,
                                     self.left_in.grip, self.left_in.trigger,
                                     self.left_in.valid),
                right=ControllerInput(self.right_in.position, self.right_in.quat_xyzw,
                                      self.right_in.grip, self.right_in.trigger,
                                      self.right_in.valid),
            )
            cur_left_grip = self._joint_positions.get(LEFT_FINGER, 0.0)
            cur_right_grip = self._joint_positions.get(RIGHT_FINGER, 0.0)

        if current_q is None:
            return
        try:
            res = self.core.step(current_q, inputs)
        except Exception as exc:  # keep the loop alive on a transient IK hiccup
            self.get_logger().warn(f"teleop step failed: {exc}", throttle_duration_sec=2.0)
            return
        if res.target_q is None:
            return

        limited = self._limit_step(res.target_q, current_q)
        left_grip = res.left_gripper if res.left_active else cur_left_grip
        right_grip = res.right_gripper if res.right_active else cur_right_grip
        self._publish(self.left_pub, limited[:7], left_grip)
        self._publish(self.right_pub, limited[7:], right_grip)

    @staticmethod
    def _publish(pub, joints, gripper):
        msg = Float64MultiArray()
        msg.data = list(np.asarray(joints, dtype=np.float64)) + [float(gripper)]
        pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OpenArmTeleopVRNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
