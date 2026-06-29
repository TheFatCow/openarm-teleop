#!/usr/bin/env python3
"""Fake VR controller UDP source for testing the OpenArm teleop pipeline.

Streams the same UDP text protocol the real Quest sender uses, so you can
exercise the bridge -> teleop -> Isaac chain with no headset. By default it
holds the grip engaged and traces a slow circle with a sinusoidal trigger.

Protocol line (relative hand):
    <LEFT|RIGHT> x y z qx qy qz qw trigger grip a b x y rate ts

Usage:
    python3 fake_udp_sender.py --hands right --ip 127.0.0.1 --port 5100
"""
import argparse
import math
import socket
import time


def line(hand, p, q, trigger, grip, ts):
    return (f"{hand} {p[0]:.5f} {p[1]:.5f} {p[2]:.5f} "
            f"{q[0]:.5f} {q[1]:.5f} {q[2]:.5f} {q[3]:.5f} "
            f"{trigger:.3f} {grip:.3f} 0 0 0 0 0.1 {ts}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ip", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5100)
    ap.add_argument("--hands", choices=["left", "right", "both"], default="right")
    ap.add_argument("--rate", type=float, default=60.0)
    ap.add_argument("--radius", type=float, default=0.08, help="circle radius (m)")
    ap.add_argument("--period", type=float, default=8.0, help="circle period (s)")
    ap.add_argument("--grip", type=float, default=1.0, help="constant grip value")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    hands = ["left", "right"] if args.hands == "both" else [args.hands]
    period = 1.0 / args.rate
    t0 = time.monotonic()
    print(f"Streaming fake {args.hands} controller(s) to {args.ip}:{args.port} "
          f"(circle r={args.radius} m). Ctrl-C to stop.")
    try:
        while True:
            t = time.monotonic() - t0
            phase = 2 * math.pi * (t / args.period)
            # controller frame deltas: circle in X/Z, identity rotation
            p = (args.radius * math.cos(phase) - args.radius,
                 0.0,
                 args.radius * math.sin(phase))
            q = (0.0, 0.0, 0.0, 1.0)
            trigger = 0.5 * (1.0 - math.cos(phase))  # 0..1 ramp
            ts = int(time.time() * 1e9)
            for h in hands:
                msg = line("LEFT" if h == "left" else "RIGHT", p, q,
                           trigger, args.grip, ts)
                sock.sendto(msg.encode(), (args.ip, args.port))
            time.sleep(period)
    except KeyboardInterrupt:
        print("\nstopped.")


if __name__ == "__main__":
    main()
