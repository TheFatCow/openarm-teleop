#!/usr/bin/env python3
"""Stream Quest 3s controllers into the OpenArm teleop bridge via SteamVR/OpenVR.

Path (all on the PC, no Android build needed):

    Quest 3s --(ALVR over Wi-Fi)--> PC SteamVR runtime --> this script --UDP--> bridge

Setup:
    1. Install SteamVR (Steam) and ALVR on the PC; install the ALVR client APK on
       the Quest 3s (sideload via SideQuest). Pair so the Quest shows up as an
       SteamVR HMD with two controllers.
    2. pip install --user openvr
    3. Start SteamVR (room-scale or standing boundary set), then run this script.

It reads both controllers' pose + trigger + grip and emits the bridge's UDP text
protocol. Raw OpenVR poses are sent through unchanged; the teleop core's relative
mode + ``axis_matrix`` handle frame alignment, so you tune mapping downstream.

Usage:
    python3 vr_steamvr_udp_sender.py --ip 127.0.0.1 --port 5100 --rate 72
"""
import argparse
import math
import socket
import time

try:
    import openvr
except ImportError:
    raise SystemExit("openvr not installed. Run: pip install --user openvr")


def mat34_to_pos_quat(m):
    """OpenVR HmdMatrix34 (3x4) -> (position xyz, quaternion xyzw)."""
    px, py, pz = m[0][3], m[1][3], m[2][3]
    # rotation 3x3 -> quaternion (Shepperd's method)
    r = [[m[0][0], m[0][1], m[0][2]],
         [m[1][0], m[1][1], m[1][2]],
         [m[2][0], m[2][1], m[2][2]]]
    tr = r[0][0] + r[1][1] + r[2][2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (r[2][1] - r[1][2]) / s
        y = (r[0][2] - r[2][0]) / s
        z = (r[1][0] - r[0][1]) / s
    elif r[0][0] > r[1][1] and r[0][0] > r[2][2]:
        s = math.sqrt(1.0 + r[0][0] - r[1][1] - r[2][2]) * 2
        w = (r[2][1] - r[1][2]) / s
        x = 0.25 * s
        y = (r[0][1] + r[1][0]) / s
        z = (r[0][2] + r[2][0]) / s
    elif r[1][1] > r[2][2]:
        s = math.sqrt(1.0 + r[1][1] - r[0][0] - r[2][2]) * 2
        w = (r[0][2] - r[2][0]) / s
        x = (r[0][1] + r[1][0]) / s
        y = 0.25 * s
        z = (r[1][2] + r[2][1]) / s
    else:
        s = math.sqrt(1.0 + r[2][2] - r[0][0] - r[1][1]) * 2
        w = (r[1][0] - r[0][1]) / s
        x = (r[0][2] + r[2][0]) / s
        y = (r[1][2] + r[2][1]) / s
        z = 0.25 * s
    return (px, py, pz), (x, y, z, w)


def read_inputs(vr, idx):
    """Return (trigger, grip) in [0,1] for controller device ``idx``.

    Axis indices follow the common SteamVR layout (trigger=axis1, grip=axis2);
    grip falls back to the digital grip button. Adjust if your binding differs.
    """
    got, state = vr.getControllerState(idx)
    if not got:
        return 0.0, 0.0
    trigger = float(state.rAxis[1].x)
    grip_axis = float(state.rAxis[2].x)
    grip_btn = 1.0 if (state.ulButtonPressed &
                       (1 << openvr.k_EButton_Grip)) else 0.0
    grip = max(grip_axis, grip_btn)
    return max(0.0, min(1.0, trigger)), max(0.0, min(1.0, grip))


def line(hand, p, q, trigger, grip, ts):
    return (f"{hand} {p[0]:.5f} {p[1]:.5f} {p[2]:.5f} "
            f"{q[0]:.5f} {q[1]:.5f} {q[2]:.5f} {q[3]:.5f} "
            f"{trigger:.3f} {grip:.3f} 0 0 0 0 0.1 {ts}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ip", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5100)
    ap.add_argument("--rate", type=float, default=72.0)
    args = ap.parse_args()

    vr = openvr.init(openvr.VRApplication_Background)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    period = 1.0 / args.rate
    roles = {
        "LEFT": openvr.TrackedControllerRole_LeftHand,
        "RIGHT": openvr.TrackedControllerRole_RightHand,
    }
    print(f"SteamVR -> UDP {args.ip}:{args.port} @ {args.rate} Hz. Ctrl-C to stop.")
    try:
        while True:
            poses = vr.getDeviceToAbsoluteTrackingPose(
                openvr.TrackingUniverseStanding, 0, openvr.k_unMaxTrackedDeviceCount)
            ts = int(time.time() * 1e9)
            for hand, role in roles.items():
                idx = vr.getTrackedDeviceIndexForControllerRole(role)
                if idx == openvr.k_unTrackedDeviceIndexInvalid:
                    continue
                pose = poses[idx]
                if not pose.bPoseIsValid:
                    continue
                p, q = mat34_to_pos_quat(pose.mDeviceToAbsoluteTracking)
                trigger, grip = read_inputs(vr, idx)
                sock.sendto(line(hand, p, q, trigger, grip, ts).encode(),
                            (args.ip, args.port))
            time.sleep(period)
    except KeyboardInterrupt:
        print("\nstopped.")
    finally:
        openvr.shutdown()


if __name__ == "__main__":
    main()
