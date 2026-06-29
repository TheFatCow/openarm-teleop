#!/usr/bin/env python3
"""Isaac Sim backend for OpenArm VR teleop (ROS-free; talks UDP/JSON).

Isaac Sim ships its own Python (3.11), which is **ABI-incompatible with ROS 2
Humble's rclpy (built for 3.10)** -- so we cannot import rclpy in this process.
Instead this script speaks a tiny localhost UDP/JSON protocol to a ROS-side
relay (``openarm_isaac_relay``, run with ``ros2 run``), which does the actual
topic pub/sub. This keeps the two Python runtimes fully decoupled.

    teleop node --(ROS topics)--> openarm_isaac_relay --(UDP/JSON)--> THIS script
    THIS script --(UDP/JSON joint_states)--> relay --(/joint_states)--> teleop node

Protocol (newline-free JSON datagrams on localhost):
  * relay -> sim   (cmd_port,  default 6001): {"left":[7 joints + gripper] | null,
                                               "right":[...] | null}
  * sim   -> relay (state_port, default 6002): {"name":[...], "position":[...],
                                                "velocity":[...]}

Run (source ROS + workspace overlay in the same shell so paths resolve)::

    ~/IsaacLab/isaaclab.sh -p \
        ~/ROS_WS/src/openarm_teleop_vr/isaac/openarm_isaac_teleop.py --headless
"""

import argparse

from isaaclab.app import AppLauncher

LOCAL_USD = "/home/student/ROS_WS/openarm_v10_bimanual/openarm_v10_bimanual.usd"

parser = argparse.ArgumentParser(description="OpenArm Isaac Sim teleop backend")
parser.add_argument("--usd", default=LOCAL_USD,
                    help="OpenArm bimanual USD (default: local converted asset)")
parser.add_argument("--use_nucleus", action="store_true",
                    help="Use the Nucleus USD from OPENARM_BI_CFG instead of --usd")
parser.add_argument("--relay_host", default="127.0.0.1")
parser.add_argument("--cmd_port", type=int, default=6001, help="UDP port: recv commands")
parser.add_argument("--state_port", type=int, default=6002, help="UDP port: send states")
parser.add_argument("--rate", type=float, default=120.0, help="sim/publish rate (Hz)")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# ---- everything below requires the running app ----------------------------
import json  # noqa: E402
import socket  # noqa: E402
import threading  # noqa: E402

import numpy as np  # noqa: E402
import torch  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.assets import Articulation  # noqa: E402
from isaaclab_assets.robots.openarm import OPENARM_BI_CFG  # noqa: E402

LEFT_JOINTS = [f"openarm_left_joint{i}" for i in range(1, 8)]
RIGHT_JOINTS = [f"openarm_right_joint{i}" for i in range(1, 8)]
LEFT_FINGERS = ["openarm_left_finger_joint1", "openarm_left_finger_joint2"]
RIGHT_FINGERS = ["openarm_right_finger_joint1", "openarm_right_finger_joint2"]


class CommandLink:
    """Receives command datagrams; sends joint-state datagrams. No ROS."""

    def __init__(self, host, cmd_port, state_port):
        self._lock = threading.Lock()
        self.left_cmd = None
        self.right_cmd = None
        self._state_addr = (host, state_port)
        self._tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rx.bind(("0.0.0.0", cmd_port))
        self._running = True
        threading.Thread(target=self._recv_loop, daemon=True).start()

    def _recv_loop(self):
        while self._running:
            try:
                data, _ = self._rx.recvfrom(8192)
            except OSError:
                break
            try:
                msg = json.loads(data.decode())
            except (ValueError, UnicodeDecodeError):
                continue
            with self._lock:
                if msg.get("left") is not None:
                    self.left_cmd = np.asarray(msg["left"], dtype=np.float64)
                if msg.get("right") is not None:
                    self.right_cmd = np.asarray(msg["right"], dtype=np.float64)

    def latest(self):
        with self._lock:
            return (None if self.left_cmd is None else self.left_cmd.copy(),
                    None if self.right_cmd is None else self.right_cmd.copy())

    def send_states(self, names, positions, velocities):
        payload = json.dumps({
            "name": list(names),
            "position": [float(p) for p in positions],
            "velocity": [float(v) for v in velocities],
        }).encode()
        try:
            self._tx.sendto(payload, self._state_addr)
        except OSError:
            pass


def design_scene():
    sim_utils.GroundPlaneCfg().func("/World/defaultGroundPlane", sim_utils.GroundPlaneCfg())
    sim_utils.DomeLightCfg(intensity=2500.0, color=(0.75, 0.75, 0.75)).func(
        "/World/Light", sim_utils.DomeLightCfg(intensity=2500.0))
    robot_cfg = OPENARM_BI_CFG.copy()
    robot_cfg.prim_path = "/World/OpenArm"
    if not args_cli.use_nucleus:
        robot_cfg.spawn.usd_path = args_cli.usd
        print(f"[openarm-isaac] spawning local USD: {args_cli.usd}")
    return Articulation(robot_cfg)


def main():
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device, dt=1.0 / args_cli.rate)
    sim = sim_utils.SimulationContext(sim_cfg)
    sim.set_camera_view([2.0, 1.5, 1.6], [0.0, 0.0, 0.9])

    robot = design_scene()
    sim.reset()

    arm_ids, _ = robot.find_joints(LEFT_JOINTS + RIGHT_JOINTS, preserve_order=True)
    lfinger_ids, _ = robot.find_joints(LEFT_FINGERS, preserve_order=True)
    rfinger_ids, _ = robot.find_joints(RIGHT_FINGERS, preserve_order=True)
    arm_ids = torch.tensor(arm_ids, device=sim.device, dtype=torch.long)
    lfinger_ids = torch.tensor(lfinger_ids, device=sim.device, dtype=torch.long)
    rfinger_ids = torch.tensor(rfinger_ids, device=sim.device, dtype=torch.long)
    joint_names = list(robot.data.joint_names)
    print(f"[openarm-isaac] {robot.num_joints} joints; arm idx {arm_ids.tolist()}")

    link = CommandLink(args_cli.relay_host, args_cli.cmd_port, args_cli.state_port)
    print(f"[openarm-isaac] UDP link up (recv cmd :{args_cli.cmd_port}, "
          f"send state -> {args_cli.relay_host}:{args_cli.state_port})")

    sim_dt = sim.get_physics_dt()
    target = robot.data.default_joint_pos.clone()  # (1, num_joints)

    while simulation_app.is_running():
        left_cmd, right_cmd = link.latest()
        if left_cmd is not None and left_cmd.shape[0] >= 7:
            target[0, arm_ids[:7]] = torch.tensor(left_cmd[:7], device=sim.device,
                                                  dtype=target.dtype)
            if left_cmd.shape[0] >= 8:
                target[0, lfinger_ids] = float(left_cmd[7])
        if right_cmd is not None and right_cmd.shape[0] >= 7:
            target[0, arm_ids[7:]] = torch.tensor(right_cmd[:7], device=sim.device,
                                                  dtype=target.dtype)
            if right_cmd.shape[0] >= 8:
                target[0, rfinger_ids] = float(right_cmd[7])

        robot.set_joint_position_target(target)
        robot.write_data_to_sim()
        sim.step()
        robot.update(sim_dt)

        link.send_states(joint_names,
                         robot.data.joint_pos[0].cpu().numpy(),
                         robot.data.joint_vel[0].cpu().numpy())


if __name__ == "__main__":
    main()
    simulation_app.close()
