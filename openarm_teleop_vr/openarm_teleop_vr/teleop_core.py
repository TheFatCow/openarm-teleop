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


def _matrix_to_quat_wxyz(R: np.ndarray) -> np.ndarray:
    q = pin.Quaternion(R)
    q.normalize()
    return np.array([q.w, q.x, q.y, q.z], dtype=np.float64)


def _quat_wxyz_to_matrix(q: np.ndarray) -> np.ndarray:
    return pin.Quaternion(q[0], q[1], q[2], q[3]).matrix()


def _quat_slerp(q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
    """Shortest-path SLERP between two (w,x,y,z) quaternions."""
    q0 = q0 / np.linalg.norm(q0)
    q1 = q1 / np.linalg.norm(q1)
    dot = float(np.dot(q0, q1))
    if dot < 0.0:            # take the shorter arc
        q1 = -q1
        dot = -dot
    if dot > 0.9995:         # nearly identical -> linear + renormalize
        q = q0 + t * (q1 - q0)
        return q / np.linalg.norm(q)
    theta = np.arccos(np.clip(dot, -1.0, 1.0))
    s0 = np.sin((1.0 - t) * theta) / np.sin(theta)
    s1 = np.sin(t * theta) / np.sin(theta)
    return s0 * q0 + s1 * q1


class _LPFilter:
    """First-order low-pass on a vector. Mirrors PAPRLE's LPFilter."""

    def __init__(self, alpha: float):
        self.alpha = float(alpha)
        self.y = None

    def next(self, x: np.ndarray) -> np.ndarray:
        x = np.asarray(x, dtype=np.float64)
        if self.y is None:
            self.y = x.copy()
        else:
            self.y = self.y + self.alpha * (x - self.y)
        return self.y.copy()

    def reset(self):
        self.y = None


class _LPRotationFilter:
    """SLERP low-pass on a rotation. Mirrors PAPRLE's LPRotationFilter."""

    def __init__(self, alpha: float):
        self.alpha = float(alpha)
        self.q = None  # (w,x,y,z)

    def next(self, R: np.ndarray) -> np.ndarray:
        q = _matrix_to_quat_wxyz(R)
        if self.q is None:
            self.q = q
        else:
            self.q = _quat_slerp(self.q, q, self.alpha)
        return _quat_wxyz_to_matrix(self.q)

    def reset(self):
        self.q = None


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
    max_iters: int = 50
    eps: float = 1e-3          # convergence threshold on the 6D log error norm
    damp: float = 1e-6         # damped-least-squares lambda^2
    dt: float = 1.0            # integration step on the velocity update
    v_max: float = 2.0         # clamp on the joint-velocity update norm (rad)
    # Random restarts MUST stay 0 for teleop: a restart can return a completely
    # different arm configuration than the seed, so consecutive ticks jump
    # between solution branches and the arm visibly contorts. Best-effort from
    # the current configuration is always the right answer while tracking.
    restarts: int = 0


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
    def __init__(self, pos_alpha: float, rot_alpha: float):
        self.engaged = False
        self.ee_target: Optional[pin.SE3] = None   # accumulated EE goal (base frame)
        self.past_p: Optional[np.ndarray] = None    # previous controller position (mapped)
        self.past_R: Optional[np.ndarray] = None    # previous controller rotation (mapped)
        self.last_q = np.zeros(7, dtype=np.float64)
        self.gripper = 0.0
        self.pos_filter = _LPFilter(pos_alpha)
        self.rot_filter = _LPRotationFilter(rot_alpha)

    def reset_filters(self):
        self.pos_filter.reset()
        self.rot_filter.reset()


class OpenArmTeleopCore:
    """Bimanual relative teleop: controller poses -> 14 joint targets + grippers.

    Delta handling follows PAPRLE's ``oculus.py``: instead of a single world-frame
    anchor delta, each tick's controller motion is measured **relative to the
    previous frame, in the controller's own (body) frame**, and composed onto the
    EE goal in its local frame (``ee_target = ee_target * delta``). Translation and
    rotation are low-pass / SLERP filtered to kill controller jitter, and the
    "previous controller pose" keeps updating even while disengaged so releasing
    and re-gripping never jumps (clutching). Body-frame deltas track the way the
    hand moves *relative to how it is held*, which is far more intuitive than
    world-frame deltas when the operator's wrist orientation drifts.
    """

    def __init__(
        self,
        urdf_path: str,
        *,
        grip_threshold: float = 0.5,
        position_scale_xyz: Sequence[float] = (1.0, 1.0, 1.0),
        axis_matrix: Optional[np.ndarray] = None,
        gripper_max: float = 0.044,
        ik_cfg: Optional[IKConfig] = None,
        pos_filter_alpha: float = 0.5,
        rot_filter_alpha: float = 0.5,
    ):
        full = pin.buildModelFromUrdf(urdf_path)
        ik_cfg = ik_cfg or IKConfig()
        self.left = _Arm(full, LEFT_JOINTS, LEFT_TCP_FRAME, ik_cfg)
        self.right = _Arm(full, RIGHT_JOINTS, RIGHT_TCP_FRAME, ik_cfg)

        self.grip_threshold = float(grip_threshold)
        self.position_scale = np.asarray(position_scale_xyz, dtype=np.float64)
        # axis_matrix maps the controller frame -> robot base frame (change of basis)
        self.axis_matrix = (np.eye(3) if axis_matrix is None
                            else np.asarray(axis_matrix, dtype=np.float64).reshape(3, 3))
        self.gripper_max = float(gripper_max)

        self._state = {
            "left": _ArmTeleopState(pos_filter_alpha, rot_filter_alpha),
            "right": _ArmTeleopState(pos_filter_alpha, rot_filter_alpha),
        }

    # -- per-arm relative-teleop update ------------------------------------
    def _mapped_controller(self, ctrl: ControllerInput) -> tuple[np.ndarray, np.ndarray]:
        """Controller pose expressed in the robot base frame (filtered)."""
        A = self.axis_matrix
        R = A @ quat_xyzw_to_matrix(*ctrl.quat_xyzw) @ A.T
        p = A @ np.asarray(ctrl.position, dtype=np.float64).reshape(3)
        return p, R

    def _step_arm(self, arm: _Arm, st: _ArmTeleopState, ctrl: ControllerInput,
                  q_current: np.ndarray) -> tuple[np.ndarray, float, bool]:
        st.last_q = np.array(q_current, dtype=np.float64).copy()
        engaged_now = ctrl.valid and ctrl.grip > self.grip_threshold

        # Always update the (filtered) current controller pose in base frame, even
        # while disengaged -- this is the clutch: re-gripping continues from here.
        if ctrl.valid:
            p_raw, R_raw = self._mapped_controller(ctrl)
            p_c = st.pos_filter.next(p_raw)
            R_c = st.rot_filter.next(R_raw)
        else:
            p_c, R_c = st.past_p, st.past_R

        if not engaged_now:
            st.engaged = False
            st.past_p, st.past_R = p_c, R_c
            # hold the last commanded goal (or current pose if never engaged)
            return q_current.copy(), st.gripper, False

        if not st.engaged or st.ee_target is None or st.past_p is None:
            # rising edge -> anchor the EE goal at the current pose; no motion yet
            st.engaged = True
            st.ee_target = arm.fk(q_current)
            st.past_p, st.past_R = p_c, R_c
            st.gripper = float(np.clip(ctrl.trigger, 0.0, 1.0)) * self.gripper_max
            return q_current.copy(), st.gripper, True

        # body-frame incremental motion since last frame (PAPRLE oculus.py):
        #   new_pos = past_R^T (p_now - p_past)   (translation delta in controller frame)
        #   new_rot = past_R^T R_now              (rotation delta in controller frame)
        new_pos = st.past_R.T @ (self.position_scale * (p_c - st.past_p))
        new_rot = st.past_R.T @ R_c
        delta = pin.SE3(new_rot, new_pos)
        st.ee_target = st.ee_target * delta      # compose in the EE's local frame

        q_target, _ = arm.ik(st.ee_target, q_current)
        st.past_p, st.past_R = p_c, R_c
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
            st.ee_target = None
            st.past_p = None
            st.past_R = None
            st.reset_filters()
