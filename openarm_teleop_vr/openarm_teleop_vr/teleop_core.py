#!/usr/bin/env python3
"""Open Pinocchio-based bimanual teleop core for the OpenArm v1.0 robot.

This is a clean-room replacement for the closed ``PinocchioTeleopCore`` that the
original OpenArmX node imported. It has **no ROS dependency** so it can be unit
tested offline (see ``test/test_teleop_core.py``).

Pipeline (MVP, relative / "grip-to-engage" mode):

* Build a full Pinocchio model from the bimanual URDF, then two *reduced* models
  (one per arm) with everything but that arm's 7 revolute joints locked.
* While a controller's grip is held, anchor the controller pose and the current
  end-effector (TCP) pose. As the controller moves, command
  ``EE_target = Delta_controller applied to EE_anchor`` and solve damped
  least-squares IK for that arm's 7 joints.
* Releasing the grip holds the last commanded joint configuration.
* The trigger maps linearly to gripper opening in metres.

The controller-frame -> robot-base-frame mapping is a tunable 3x3 ``axis_matrix``
(should be a proper rotation) plus a per-axis ``position_scale_xyz``.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, Sequence

import numpy as np
import pinocchio as pin

LEFT_JOINTS = [f"openarm_left_joint{i}" for i in range(1, 8)]
RIGHT_JOINTS = [f"openarm_right_joint{i}" for i in range(1, 8)]
LEFT_TCP_FRAME = "openarm_left_hand_tcp"
RIGHT_TCP_FRAME = "openarm_right_hand_tcp"


def quat_xyzw_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Return the 3x3 rotation matrix for a (x, y, z, w) quaternion."""
    q = np.array([x, y, z, w], dtype=np.float64)
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.eye(3)
    q = q / n
    return pin.Quaternion(q[3], q[0], q[1], q[2]).matrix()


@dataclass
class ControllerInput:
    """One controller's instantaneous state."""

    position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    quat_xyzw: tuple = (0.0, 0.0, 0.0, 1.0)
    grip: float = 0.0
    trigger: float = 0.0
    valid: bool = False  # True once a pose has actually been received


@dataclass
class TeleopInputs:
    left: ControllerInput = field(default_factory=ControllerInput)
    right: ControllerInput = field(default_factory=ControllerInput)


@dataclass
class TeleopResult:
    target_q: Optional[np.ndarray]  # shape (14,) = left7 + right7, or None if no joint states yet
    left_gripper: float = 0.0       # metres
    right_gripper: float = 0.0      # metres
    left_active: bool = False       # arm currently being driven by the controller
    right_active: bool = False
    waiting_for_joint_states: bool = False


@dataclass
class IKConfig:
    max_iters: int = 100
    eps: float = 1e-4          # convergence threshold on the 6D log error norm
    damp: float = 1e-6         # damped-least-squares lambda^2
    dt: float = 1.0            # integration step on the velocity update
    v_max: float = 2.0         # clamp on the joint-velocity update norm (rad)
    restarts: int = 4          # random restarts if the seed fails to converge


class _Arm:
    """A reduced single-arm Pinocchio model with IK/FK helpers."""

    def __init__(self, full_model: pin.Model, keep_joints: Sequence[str],
                 tcp_frame: str, ik_cfg: IKConfig):
        lock_ids = [
            jid for jid in range(1, full_model.njoints)
            if full_model.names[jid] not in keep_joints
        ]
        q_ref = pin.neutral(full_model)
        self.model = pin.buildReducedModel(full_model, lock_ids, q_ref)
        self.data = self.model.createData()
        if not self.model.existFrame(tcp_frame):
            raise ValueError(f"TCP frame '{tcp_frame}' not found in reduced model")
        self.tcp_id = self.model.getFrameId(tcp_frame)
        self.q_lower = self.model.lowerPositionLimit.copy()
        self.q_upper = self.model.upperPositionLimit.copy()
        self.ik_cfg = ik_cfg
        assert self.model.nq == 7, f"expected 7 DoF arm, got {self.model.nq}"

    def fk(self, q: np.ndarray) -> pin.SE3:
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        return self.data.oMf[self.tcp_id].copy()

    def _ik_from(self, target: pin.SE3, q_seed: np.ndarray) -> tuple[np.ndarray, float]:
        """Single damped-least-squares solve from one seed. Returns (q, final_err)."""
        cfg = self.ik_cfg
        q = np.clip(np.array(q_seed, dtype=np.float64), self.q_lower, self.q_upper)
        err_norm = np.inf
        for _ in range(cfg.max_iters):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)
            oMf = self.data.oMf[self.tcp_id]
            iMd = oMf.actInv(target)            # current -> desired, in local TCP frame
            err = pin.log6(iMd).vector          # 6D spatial error
            err_norm = float(np.linalg.norm(err))
            if err_norm < cfg.eps:
                break
            J = pin.computeFrameJacobian(self.model, self.data, q, self.tcp_id,
                                         pin.ReferenceFrame.LOCAL)
            J = -np.dot(pin.Jlog6(iMd.inverse()), J)
            JJt = J @ J.T + cfg.damp * np.eye(6)
            v = -J.T @ np.linalg.solve(JJt, err)
            v_norm = np.linalg.norm(v)
            if v_norm > cfg.v_max:              # avoid huge steps near singularities
                v = v * (cfg.v_max / v_norm)
            q = np.clip(pin.integrate(self.model, q, v * cfg.dt),
                        self.q_lower, self.q_upper)
        return q, err_norm

    def ik(self, target: pin.SE3, q_seed: np.ndarray) -> tuple[np.ndarray, bool]:
        """IK toward ``target`` (an SE3 in the model root frame).

        Tries the provided seed first (the realistic teleop case: seed == current
        q is always close), then random restarts as a fallback. Returns
        ``(q, success)``; ``q`` is always within joint limits, best-effort on fail.
        """
        cfg = self.ik_cfg
        best_q, best_err = self._ik_from(target, q_seed)
        if best_err < cfg.eps:
            return best_q, True
        rng = np.random.default_rng(0)
        span = self.q_upper - self.q_lower
        for _ in range(cfg.restarts):
            seed = self.q_lower + span * rng.random(self.model.nq)
            q, err = self._ik_from(target, seed)
            if err < best_err:
                best_q, best_err = q, err
            if best_err < cfg.eps:
                return best_q, True
        return best_q, best_err < cfg.eps


class _ArmTeleopState:
    def __init__(self):
        self.engaged = False
        self.ctrl_anchor: Optional[pin.SE3] = None
        self.ee_anchor: Optional[pin.SE3] = None
        self.last_q = np.zeros(7, dtype=np.float64)
        self.gripper = 0.0


class OpenArmTeleopCore:
    """Bimanual relative teleop: controller poses -> 14 joint targets + grippers."""

    def __init__(
        self,
        urdf_path: str,
        *,
        grip_threshold: float = 0.5,
        position_scale_xyz: Sequence[float] = (1.0, 1.0, 1.0),
        axis_matrix: Optional[np.ndarray] = None,
        gripper_max: float = 0.044,
        ik_cfg: Optional[IKConfig] = None,
    ):
        full = pin.buildModelFromUrdf(urdf_path)
        ik_cfg = ik_cfg or IKConfig()
        self.left = _Arm(full, LEFT_JOINTS, LEFT_TCP_FRAME, ik_cfg)
        self.right = _Arm(full, RIGHT_JOINTS, RIGHT_TCP_FRAME, ik_cfg)

        self.grip_threshold = float(grip_threshold)
        self.position_scale = np.asarray(position_scale_xyz, dtype=np.float64)
        self.axis_matrix = (np.eye(3) if axis_matrix is None
                            else np.asarray(axis_matrix, dtype=np.float64).reshape(3, 3))
        self.gripper_max = float(gripper_max)

        self._state = {"left": _ArmTeleopState(), "right": _ArmTeleopState()}

    # -- per-arm relative-teleop update ------------------------------------
    def _controller_se3(self, ctrl: ControllerInput) -> pin.SE3:
        R = quat_xyzw_to_matrix(*ctrl.quat_xyzw)
        return pin.SE3(R, np.asarray(ctrl.position, dtype=np.float64).reshape(3))

    def _target_ee(self, ctrl_now: pin.SE3, st: _ArmTeleopState) -> pin.SE3:
        """Map the controller delta (since engage) onto the anchored EE pose."""
        A = self.axis_matrix
        # translation delta in controller/world frame -> robot base frame
        dp_world = ctrl_now.translation - st.ctrl_anchor.translation
        dp = A @ (self.position_scale * dp_world)
        # rotation delta, conjugated into the robot base frame
        dR_world = ctrl_now.rotation @ st.ctrl_anchor.rotation.T
        dR = A @ dR_world @ A.T
        target_R = dR @ st.ee_anchor.rotation
        target_p = st.ee_anchor.translation + dp
        return pin.SE3(target_R, target_p)

    def _step_arm(self, arm: _Arm, st: _ArmTeleopState, ctrl: ControllerInput,
                  q_current: np.ndarray) -> tuple[np.ndarray, float, bool]:
        st.last_q = np.array(q_current, dtype=np.float64).copy()
        engaged_now = ctrl.valid and ctrl.grip > self.grip_threshold

        if not engaged_now:
            st.engaged = False
            return q_current.copy(), st.gripper, False

        ctrl_se3 = self._controller_se3(ctrl)
        if not st.engaged:
            # rising edge -> anchor controller pose and current EE pose
            st.engaged = True
            st.ctrl_anchor = ctrl_se3
            st.ee_anchor = arm.fk(q_current)

        target = self._target_ee(ctrl_se3, st)
        q_target, _ = arm.ik(target, q_current)
        st.gripper = float(np.clip(ctrl.trigger, 0.0, 1.0)) * self.gripper_max
        return q_target, st.gripper, True

    # -- public API ---------------------------------------------------------
    def step(self, current_q14: Optional[np.ndarray], inputs: TeleopInputs) -> TeleopResult:
        if current_q14 is None:
            return TeleopResult(target_q=None, waiting_for_joint_states=True)
        current_q14 = np.asarray(current_q14, dtype=np.float64).reshape(14)

        ql, gl, al = self._step_arm(self.left, self._state["left"],
                                    inputs.left, current_q14[:7])
        qr, gr, ar = self._step_arm(self.right, self._state["right"],
                                    inputs.right, current_q14[7:])
        return TeleopResult(
            target_q=np.concatenate([ql, qr]),
            left_gripper=gl, right_gripper=gr,
            left_active=al, right_active=ar,
        )

    def reset(self):
        for st in self._state.values():
            st.engaged = False
            st.ctrl_anchor = None
            st.ee_anchor = None
