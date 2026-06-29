#!/usr/bin/env python3
"""Offline validation of the open teleop core (no ROS, no sim).

Run directly:  python3 test/test_teleop_core.py
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


def _rand_q(arm, rng, margin=0.1):
    lo, hi = arm.q_lower + margin, arm.q_upper - margin
    return lo + (hi - lo) * rng.random(arm.model.nq)


def test_ik_roundtrip():
    core = OpenArmTeleopCore(URDF, ik_cfg=IKConfig(max_iters=300, eps=1e-5))
    rng = np.random.default_rng(0)
    for name, arm in (("left", core.left), ("right", core.right)):
        n_ok = 0
        n_trials = 25
        for _ in range(n_trials):
            q_true = _rand_q(arm, rng)
            target = arm.fk(q_true)
            seed = _rand_q(arm, rng)  # deliberately far seed
            q_sol, ok = arm.ik(target, seed)
            achieved = arm.fk(q_sol)
            err = np.linalg.norm(pin.log6(achieved.actInv(target)).vector)
            within = bool(np.all(q_sol >= arm.q_lower - 1e-9) and
                          np.all(q_sol <= arm.q_upper + 1e-9))
            assert within, f"{name}: IK solution violated joint limits"
            if err < 1e-3:
                n_ok += 1
        # 7-DoF arm is redundant; with random far seeds most should converge.
        assert n_ok >= int(0.8 * n_trials), f"{name}: only {n_ok}/{n_trials} IK converged"
        print(f"[ok] {name} IK round-trip: {n_ok}/{n_trials} converged < 1e-3")


def test_relative_translation():
    """Engaging the grip and translating the controller should translate the EE
    by (axis_matrix @ scale * delta), within IK error."""
    core = OpenArmTeleopCore(URDF, ik_cfg=IKConfig(max_iters=300, eps=1e-5))
    # start both arms at a comfortable mid-range posture
    q0 = np.zeros(14)
    q0[:7] = 0.5 * (core.left.q_lower + core.left.q_upper)
    q0[7:] = 0.5 * (core.right.q_lower + core.right.q_upper)

    ee0 = core.left.fk(q0[:7])

    # frame 1: grip engaged, controller at anchor (no motion yet)
    left = ControllerInput(position=np.array([0.0, 0.0, 0.0]),
                           quat_xyzw=(0, 0, 0, 1), grip=1.0, trigger=0.0, valid=True)
    res = core.step(q0, TeleopInputs(left=left))
    assert res.left_active

    # frame 2: move controller +5cm in x, +3cm in z
    delta = np.array([0.05, 0.0, 0.03])
    left2 = ControllerInput(position=delta, quat_xyzw=(0, 0, 0, 1),
                            grip=1.0, trigger=0.5, valid=True)
    res2 = core.step(q0, TeleopInputs(left=left2))
    ee_target_motion = core.left.fk(res2.target_q[:7]).translation - ee0.translation
    expected = core.axis_matrix @ (core.position_scale * delta)
    err = np.linalg.norm(ee_target_motion - expected)
    assert err < 5e-3, f"EE motion {ee_target_motion} != expected {expected} (err {err:.4f})"
    # trigger 0.5 -> half-open gripper
    assert abs(res2.left_gripper - 0.5 * core.gripper_max) < 1e-6
    print(f"[ok] relative translation: EE moved {ee_target_motion}, expected {expected}")


def test_grip_release_holds():
    core = OpenArmTeleopCore(URDF)
    q0 = np.zeros(14)
    # grip released -> arm holds current q, not active
    left = ControllerInput(position=np.zeros(3), quat_xyzw=(0, 0, 0, 1),
                           grip=0.0, trigger=0.0, valid=True)
    res = core.step(q0, TeleopInputs(left=left))
    assert not res.left_active
    assert np.allclose(res.target_q[:7], q0[:7])
    print("[ok] grip released -> holds posture, inactive")


def test_waiting_for_joint_states():
    core = OpenArmTeleopCore(URDF)
    res = core.step(None, TeleopInputs())
    assert res.waiting_for_joint_states and res.target_q is None
    print("[ok] no joint states -> waiting")


if __name__ == "__main__":
    test_ik_roundtrip()
    test_relative_translation()
    test_grip_release_holds()
    test_waiting_for_joint_states()
    print("\nAll teleop_core tests passed.")
