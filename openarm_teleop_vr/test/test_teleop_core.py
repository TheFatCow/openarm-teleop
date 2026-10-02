#!/usr/bin/env python3
"""Offline validation of the open teleop core (no ROS, no sim).

Run directly:  python3 test/test_teleop_core.py

Covers the body-frame incremental teleop scheme (PAPRLE oculus.py style):
IK from a close seed (the realistic teleop case), no drift when the controller
is still, smooth ~1:1 translation tracking, clutch (release/reposition/re-grip
without a jump), and rotation tracking.
"""
import os
import sys

import numpy as np
import pinocchio as pin

# Allow running from the package dir without installing.
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from openarm_teleop_vr.teleop_core import (  # noqa: E402
    ControllerInput, IKConfig, OpenArmTeleopCore, TeleopInputs,
)

URDF = os.environ.get("OPENARM_URDF", "/home/student/ROS_WS/openarm_v10_bimanual.urdf")

# A bent, dexterous home posture (matches the Isaac spawn pose). The all-zeros
# pose is a kinematic singularity, so IK/teleop must never be tested from there.
RIGHT_HOME = np.array([0.0, 0.6, 0.0, 1.2, 0.0, 0.5, 0.0])
LEFT_HOME = np.array([0.0, -0.6, 0.0, 1.2, 0.0, -0.5, 0.0])
AXIS = np.array([0, 0, -1, -1, 0, 0, 0, 1, 0], dtype=float).reshape(3, 3)


def _home_q14():
    q = np.zeros(14)
    q[:7] = LEFT_HOME
    q[7:] = RIGHT_HOME
    return q


def test_ik_close_seed():
    """Realistic teleop IK: small target perturbation, seeded from current q.
    With restarts disabled (teleop must not branch-jump), a close seed must
    still converge essentially every time."""
    core = OpenArmTeleopCore(URDF, ik_cfg=IKConfig())
    rng = np.random.default_rng(0)
    for name, arm, home in (("left", core.left, LEFT_HOME),
                            ("right", core.right, RIGHT_HOME)):
        n_ok = 0
        n_trials = 30
        for _ in range(n_trials):
            q_seed = home + rng.uniform(-0.15, 0.15, 7)
            q_seed = np.clip(q_seed, arm.q_lower, arm.q_upper)
            # target = FK of a small further perturbation (reachable, nearby)
            q_true = np.clip(q_seed + rng.uniform(-0.1, 0.1, 7), arm.q_lower, arm.q_upper)
            target = arm.fk(q_true)
            q_sol, _ = arm.ik(target, q_seed)
            err = np.linalg.norm(pin.log6(arm.fk(q_sol).actInv(target)).vector)
            assert np.all(q_sol >= arm.q_lower - 1e-9) and np.all(q_sol <= arm.q_upper + 1e-9), \
                f"{name}: IK violated joint limits"
            if err < 1e-3:
                n_ok += 1
        assert n_ok >= int(0.9 * n_trials), f"{name}: only {n_ok}/{n_trials} close-seed IK converged"
        print(f"[ok] {name} close-seed IK: {n_ok}/{n_trials} converged")


def _ctrl(p, quat=(0, 0, 0, 1), grip=1.0, trigger=0.0):
    return ControllerInput(position=np.array(p, dtype=float), quat_xyzw=quat,
                           grip=grip, trigger=trigger, valid=True)


def test_still_no_drift():
    """Grip held, controller perfectly still -> EE must not drift."""
    core = OpenArmTeleopCore(URDF, axis_matrix=AXIS)
    q = _home_q14()
    p = [0.10, 0.05, -0.03]
    ee0 = None
    for i in range(60):
        res = core.step(q, TeleopInputs(right=_ctrl(p)))
        q[7:] = res.target_q[7:]
        if i == 1:
            ee0 = core.right.fk(q[7:]).translation.copy()
    ee1 = core.right.fk(q[7:]).translation
    drift = np.linalg.norm(ee1 - ee0)
    assert drift < 1e-3, f"EE drifted {drift*1000:.2f}mm with a still controller"
    print(f"[ok] still controller -> no drift ({drift*1000:.4f}mm over 60 ticks)")


def test_smooth_translation_tracks():
    """Smoothly translating the controller moves the EE proportionally and
    monotonically (no wild jumps)."""
    core = OpenArmTeleopCore(URDF, axis_matrix=AXIS, position_scale_xyz=[1, 1, 1])
    q = _home_q14()
    ee0 = core.right.fk(q[7:]).translation.copy()
    steps = []
    prev = ee0
    for i in range(101):
        p = [0.001 * i, 0.0, 0.0]        # 0.1 m ramp over 100 ticks
        res = core.step(q, TeleopInputs(right=_ctrl(p)))
        q[7:] = res.target_q[7:]
        ee = core.right.fk(q[7:]).translation
        steps.append(np.linalg.norm(ee - prev)); prev = ee
    moved = np.linalg.norm(core.right.fk(q[7:]).translation - ee0)
    max_step = max(steps)
    assert 0.05 < moved < 0.15, f"10cm controller move -> {moved*1000:.0f}mm EE (want ~100mm)"
    assert max_step < 0.01, f"non-smooth: a single tick jumped {max_step*1000:.1f}mm"
    print(f"[ok] smooth 10cm translation -> {moved*1000:.0f}mm EE, max tick {max_step*1000:.2f}mm")


def test_clutch_no_jump():
    """Engage, move, RELEASE, reposition the hand far away, RE-GRIP: the EE must
    resume from where it was, not jump by the reposition distance."""
    core = OpenArmTeleopCore(URDF, axis_matrix=AXIS)
    q = _home_q14()
    # engage at origin, move to +8cm x
    for i in range(40):
        res = core.step(q, TeleopInputs(right=_ctrl([0.001 * min(i, 30), 0, 0])))
        q[7:] = res.target_q[7:]
    ee_before_release = core.right.fk(q[7:]).translation.copy()
    # release grip, move hand far away (30cm) while disengaged
    for i in range(20):
        res = core.step(q, TeleopInputs(right=_ctrl([0.03 + 0.01 * i, 0.2, -0.1], grip=0.0)))
        q[7:] = res.target_q[7:]
    ee_after_release = core.right.fk(q[7:]).translation.copy()
    hold_drift = np.linalg.norm(ee_after_release - ee_before_release)
    # re-grip at the new hand location: first engaged tick must NOT jump the EE
    res = core.step(q, TeleopInputs(right=_ctrl([0.23, 0.2, -0.1], grip=1.0)))
    q[7:] = res.target_q[7:]
    ee_regrip = core.right.fk(q[7:]).translation
    jump = np.linalg.norm(ee_regrip - ee_after_release)
    assert hold_drift < 1e-3, f"EE drifted {hold_drift*1000:.1f}mm while grip released"
    assert jump < 5e-3, f"EE jumped {jump*1000:.1f}mm on re-grip (clutch broken)"
    print(f"[ok] clutch: held during release ({hold_drift*1000:.3f}mm), no jump on re-grip ({jump*1000:.3f}mm)")


def test_rotation_tracks():
    """Rotating the controller (grip held) rotates the EE without exploding
    position."""
    core = OpenArmTeleopCore(URDF, axis_matrix=AXIS)
    q = _home_q14()
    R0 = core.right.fk(q[7:]).rotation.copy()
    p0 = core.right.fk(q[7:]).translation.copy()
    # engage, then rotate controller about its z by up to ~40deg over 60 ticks
    for i in range(61):
        ang = np.deg2rad(40.0) * (i / 60.0)
        quat = (0.0, 0.0, np.sin(ang / 2), np.cos(ang / 2))
        res = core.step(q, TeleopInputs(right=_ctrl([0, 0, 0], quat=quat)))
        q[7:] = res.target_q[7:]
    R1 = core.right.fk(q[7:]).rotation
    rot_change = np.linalg.norm(pin.log3(R0.T @ R1))
    pos_change = np.linalg.norm(core.right.fk(q[7:]).translation - p0)
    assert rot_change > np.deg2rad(10), f"EE barely rotated ({np.rad2deg(rot_change):.1f}deg) for a 40deg controller twist"
    assert pos_change < 0.1, f"pure rotation caused {pos_change*1000:.0f}mm position drift"
    print(f"[ok] rotation: 40deg twist -> {np.rad2deg(rot_change):.0f}deg EE rotation, {pos_change*1000:.0f}mm position drift")


def test_grip_release_holds():
    core = OpenArmTeleopCore(URDF, axis_matrix=AXIS)
    q = _home_q14()
    res = core.step(q, TeleopInputs(right=_ctrl([0, 0, 0], grip=0.0)))
    assert not res.right_active
    assert np.allclose(res.target_q[7:], q[7:])
    print("[ok] grip released -> holds posture, inactive")


def test_waiting_for_joint_states():
    core = OpenArmTeleopCore(URDF, axis_matrix=AXIS)
    res = core.step(None, TeleopInputs())
    assert res.waiting_for_joint_states and res.target_q is None
    print("[ok] no joint states -> waiting")


if __name__ == "__main__":
    test_ik_close_seed()
    test_still_no_drift()
    test_smooth_translation_tracks()
    test_clutch_no_jump()
    test_rotation_tracks()
    test_grip_release_holds()
    test_waiting_for_joint_states()
    print("\nAll teleop_core tests passed.")
