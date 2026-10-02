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
parser.add_argument("--render_decimation", type=int, default=4,
                    help="render one GUI frame every N physics steps. Rendering every "
                         "step (1) caps physics at the ~20 FPS render rate, so sim time "
                         "runs ~6x slower than real time.")
parser.add_argument("--camera", action="store_true",
                    help="mount a robot's-perspective camera and serve it as MJPEG over "
                         "HTTP (view in any browser, incl. the Quest browser)")
parser.add_argument("--cam_port", type=int, default=8080, help="MJPEG HTTP port")
parser.add_argument("--cam_w", type=int, default=640)
parser.add_argument("--cam_h", type=int, default=360)
parser.add_argument("--cam_decimation", type=int, default=8,
                    help="render/publish the camera every N physics steps (keeps FPS up)")
parser.add_argument("--cam_eye", default="0.15,0.0,1.05",
                    help="camera position x,y,z (robot head area)")
parser.add_argument("--cam_target", default="0.38,0.0,0.20",
                    help="camera look-at point x,y,z (the gripper workspace)")
parser.add_argument("--cam_fov", type=float, default=24.0,
                    help="camera focal length (mm); smaller = wider FOV")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# Camera sensors only render if the app is launched with cameras enabled.
if args_cli.camera:
    args_cli.enable_cameras = True

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# ---- everything below requires the running app ----------------------------
import json  # noqa: E402
import socket  # noqa: E402
import threading  # noqa: E402
import time  # noqa: E402
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer  # noqa: E402

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.assets import Articulation  # noqa: E402
from isaaclab_assets.robots.openarm import OPENARM_BI_HIGH_PD_CFG as OPENARM_BI_CFG  # noqa: E402


class MJPEGServer:
    """Serves the latest camera frame as multipart MJPEG over HTTP (any browser)."""

    def __init__(self, port, width, height):
        self._lock = threading.Lock()
        self._jpeg = None
        self.port = port
        self._placeholder(width, height)
        srv = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def do_GET(self):
                if self.path not in ("/", "/stream", "/stream.mjpg"):
                    self.send_response(404); self.end_headers(); return
                self.send_response(200)
                self.send_header("Content-Type",
                                 "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                try:
                    while True:
                        with srv._lock:
                            buf = srv._jpeg
                        if buf is not None:
                            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                             b"Content-Length: " + str(len(buf)).encode()
                                             + b"\r\n\r\n" + buf + b"\r\n")
                        time.sleep(0.05)
                except (BrokenPipeError, ConnectionResetError):
                    pass

        self._httpd = ThreadingHTTPServer(("0.0.0.0", port), Handler)
        threading.Thread(target=self._httpd.serve_forever, daemon=True).start()

    def _placeholder(self, w, h):
        img = np.zeros((h, w, 3), np.uint8)
        cv2.putText(img, "waiting for camera...", (20, h // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 200, 200), 2)
        self._jpeg = cv2.imencode(".jpg", img)[1].tobytes()

    def publish(self, rgb: np.ndarray):
        bgr = cv2.cvtColor(rgb[:, :, :3], cv2.COLOR_RGB2BGR)
        ok, buf = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            with self._lock:
                self._jpeg = buf.tobytes()
# HIGH_PD variant (stiffness=400, damping=80, gravity disabled) instead of the
# default low-PD OPENARM_BI_CFG (stiffness=80, gravity on) -- the low-PD config
# is tuned for torque-realistic control and visibly sags/undershoots commanded
# position targets under our IK-driven teleop; HIGH_PD is what Isaac Lab itself
# recommends for task-space/IK position control.

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


# Bent, dexterous "ready" home pose. The config default is all-zeros, which is a
# kinematic SINGULARITY (manipulability 0, elbow joint4 jammed at its lower limit
# of 0.0) -- from there IK can't move the arm in most directions, so teleop barely
# tracks. These poses put both arms mid-range with manipulability ~0.045. The two
# arms are mirror-symmetric (mirrored joint limits), so the left pose is the right
# pose with joint2 and joint6 negated.
RIGHT_HOME = [0.0, 0.6, 0.0, 1.2, 0.0, 0.5, 0.0]
LEFT_HOME = [0.0, -0.6, 0.0, 1.2, 0.0, -0.5, 0.0]


def design_scene():
    sim_utils.GroundPlaneCfg().func("/World/defaultGroundPlane", sim_utils.GroundPlaneCfg())
    sim_utils.DomeLightCfg(intensity=2500.0, color=(0.75, 0.75, 0.75)).func(
        "/World/Light", sim_utils.DomeLightCfg(intensity=2500.0))
    robot_cfg = OPENARM_BI_CFG.copy()
    robot_cfg.prim_path = "/World/OpenArm"
    home = {f"openarm_left_joint{i+1}": LEFT_HOME[i] for i in range(7)}
    home.update({f"openarm_right_joint{i+1}": RIGHT_HOME[i] for i in range(7)})
    home["openarm_left_finger_joint.*"] = 0.0
    home["openarm_right_finger_joint.*"] = 0.0
    robot_cfg.init_state = robot_cfg.init_state.replace(joint_pos=home)
    if not args_cli.use_nucleus:
        robot_cfg.spawn.usd_path = args_cli.usd
        print(f"[openarm-isaac] spawning local USD: {args_cli.usd}")
    return Articulation(robot_cfg)


def main():
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device, dt=1.0 / args_cli.rate)
    sim = sim_utils.SimulationContext(sim_cfg)
    sim.set_camera_view([2.0, 1.5, 1.6], [0.0, 0.0, 0.9])

    robot = design_scene()

    # Robot's-perspective camera (world-fixed at the head; the torso doesn't move).
    # Looks forward (+x) and down at the gripper workspace.
    camera = mjpeg = None
    if args_cli.camera:
        from isaaclab.sensors import Camera, CameraCfg  # noqa: E402
        cam_cfg = CameraCfg(
            prim_path="/World/head_cam",
            update_period=0.0,
            height=args_cli.cam_h, width=args_cli.cam_w,
            data_types=["rgb"],
            spawn=sim_utils.PinholeCameraCfg(focal_length=args_cli.cam_fov,
                                             clipping_range=(0.05, 20.0)),
        )
        camera = Camera(cfg=cam_cfg)

    sim.reset()

    if camera is not None:
        eye = torch.tensor([[float(v) for v in args_cli.cam_eye.split(",")]], device=sim.device)
        tgt = torch.tensor([[float(v) for v in args_cli.cam_target.split(",")]], device=sim.device)
        camera.set_world_poses_from_view(eye, tgt)
        mjpeg = MJPEGServer(args_cli.cam_port, args_cli.cam_w, args_cli.cam_h)
        print(f"[openarm-isaac] MJPEG camera on http://0.0.0.0:{args_cli.cam_port}/ "
              f"(open http://192.168.0.214:{args_cli.cam_port}/ in a browser)")

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
    render_every = max(1, int(args_cli.render_decimation))
    step_i = 0

    # Real-time pacing + per-component timing. Without pacing, sim time vs wall
    # time is whatever the loop happens to run at (historically ~20 steps/s of
    # 1/120 s steps = 6x slow motion). With pacing, each physics step consumes
    # sim_dt of wall time when the loop can keep up; the RTF report shows when
    # it can't and which component (physics/render/other) is to blame.
    t_phys = t_rend = t_other = 0.0
    n_report = 0
    next_tick = last_report = time.perf_counter()

    while simulation_app.is_running():
        t0 = time.perf_counter()
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
        t1 = time.perf_counter()
        sim.step(render=False)
        t2 = time.perf_counter()
        step_i += 1
        cam_every = max(1, int(args_cli.cam_decimation))
        did_render = step_i % render_every == 0
        # The camera needs a fresh render to update; piggyback on / force a render.
        if did_render or (camera is not None and step_i % cam_every == 0):
            sim.render()
        if camera is not None and step_i % cam_every == 0:
            camera.update(sim_dt)
            rgb = camera.data.output["rgb"]
            if rgb is not None and rgb.shape[0] > 0:
                mjpeg.publish(rgb[0].cpu().numpy())
        t3 = time.perf_counter()
        robot.update(sim_dt)

        link.send_states(joint_names,
                         robot.data.joint_pos[0].cpu().numpy(),
                         robot.data.joint_vel[0].cpu().numpy())
        t4 = time.perf_counter()

        t_other += (t1 - t0) + (t4 - t3)
        t_phys += t2 - t1
        t_rend += t3 - t2
        n_report += 1

        # pacing: aim for one physics step per sim_dt of wall time
        next_tick += sim_dt
        now = time.perf_counter()
        if next_tick > now:
            time.sleep(next_tick - now)
        else:
            next_tick = now  # behind schedule -- don't accumulate debt

        if now - last_report > 5.0:
            dt_wall = now - last_report
            rtf = n_report * sim_dt / dt_wall
            print(f"[openarm-isaac] {n_report / dt_wall:6.1f} steps/s  RTF {rtf:4.2f}  "
                  f"phys {1e3 * t_phys / n_report:5.1f}ms  "
                  f"render {1e3 * t_rend / n_report:5.1f}ms  "
                  f"other {1e3 * t_other / n_report:5.1f}ms", flush=True)
            last_report = now
            n_report = 0
            t_phys = t_rend = t_other = 0.0


if __name__ == "__main__":
    main()
    simulation_app.close()
