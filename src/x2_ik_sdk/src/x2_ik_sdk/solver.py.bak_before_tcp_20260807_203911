from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable

import numpy as np
import pinocchio as pin

from .config import ARM_POS_ORDER, LEFT_ARM_JOINTS, RIGHT_ARM_JOINTS, ArmSide, X2IKConfig


@dataclass(frozen=True)
class IKResult:
    success: bool
    side: ArmSide
    arm_pos: list[float]
    active_arm: list[float]
    target_xyz: list[float]
    final_xyz: list[float]
    error_norm: float
    iterations: int
    ee_frame: str
    message: str = ""


class X2ArmIKSolver:
    def __init__(self, config: X2IKConfig):
        self.config = config
        if not config.urdf_path.exists():
            raise FileNotFoundError(
                "URDF not found: "
                f"{config.urdf_path}\n"
                "The bundled model must exist at "
                "`src/x2_ik_sdk/resources/"
                "x2_ultra_plus_omnipicker_omnipicker.urdf`."
            )
        self.model = pin.buildModelFromUrdf(str(config.urdf_path))
        self.data = self.model.createData()
        self._validate_model()

    def solve_position(
        self,
        side: ArmSide | str,
        target_xyz: Iterable[float],
        current_arm_pos: Iterable[float] | None = None,
        *,
        current_head_pos: Iterable[float] | None = None,
        q_seed: np.ndarray | None = None,
    ) -> IKResult:
        side = ArmSide(side)
        target = np.asarray(list(target_xyz), dtype=float)
        if target.shape != (3,):
            raise ValueError(f"target_xyz must have length 3, got {target}")

        q = self._seed_q(current_arm_pos, current_head_pos, q_seed)
        frame_name = self.config.frame_for_side(side)
        frame_id = self.model.getFrameId(frame_name)
        active_v_idxs = self._active_velocity_indices(side)

        err_norm = math.inf
        iterations = 0
        success = False
        for iterations in range(1, self.config.max_iters + 1):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)

            current = self.data.oMf[frame_id].translation
            err = target - current
            err_norm = float(np.linalg.norm(err))
            if err_norm < self.config.eps:
                success = True
                break

            jacobian6 = pin.computeFrameJacobian(
                self.model,
                self.data,
                q,
                frame_id,
                pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
            )
            jacobian = jacobian6[:3, :]
            velocity = self._damped_least_squares(jacobian, err, active_v_idxs)
            step = velocity * self.config.dt
            step_norm = float(np.linalg.norm(step))
            if step_norm > self.config.max_step_norm:
                step *= self.config.max_step_norm / step_norm

            q = pin.integrate(self.model, q, step)
            q = self._clip_q(q)

        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        final = self.data.oMf[frame_id].translation.copy()
        arm_pos = self.arm_pos_from_q(q)
        active_arm = arm_pos[:7] if side == ArmSide.LEFT else arm_pos[7:]

        msg = "converged" if success else "max iterations reached"
        return IKResult(
            success=success,
            side=side,
            arm_pos=arm_pos,
            active_arm=active_arm,
            target_xyz=target.tolist(),
            final_xyz=final.tolist(),
            error_norm=err_norm,
            iterations=iterations,
            ee_frame=frame_name,
            message=msg,
        )

    def fk_xyz(
        self,
        side: ArmSide | str,
        current_arm_pos: Iterable[float] | None = None,
        *,
        current_head_pos: Iterable[float] | None = None,
        q_seed: np.ndarray | None = None,
    ) -> list[float]:
        side = ArmSide(side)
        q = self._seed_q(current_arm_pos, current_head_pos, q_seed)
        frame_id = self.model.getFrameId(self.config.frame_for_side(side))
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        return self.data.oMf[frame_id].translation.copy().tolist()

    def fk_rpy(
        self,
        side: ArmSide | str,
        current_arm_pos: Iterable[float] | None = None,
        *,
        current_head_pos: Iterable[float] | None = None,
        q_seed: np.ndarray | None = None,
    ) -> list[float]:
        side = ArmSide(side)
        q = self._seed_q(current_arm_pos, current_head_pos, q_seed)
        frame_id = self.model.getFrameId(self.config.frame_for_side(side))
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        return pin.rpy.matrixToRpy(self.data.oMf[frame_id].rotation).tolist()

    def q_from_arm_pos(
        self,
        arm_pos: Iterable[float],
        current_head_pos: Iterable[float] | None = None,
    ) -> np.ndarray:
        q = pin.neutral(self.model)
        arm_values = list(arm_pos)
        if len(arm_values) != 14:
            raise ValueError(f"arm_pos must have length 14, got {len(arm_values)}")
        for joint_name, value in zip(ARM_POS_ORDER, arm_values):
            self._set_scalar_joint(q, joint_name, value)
        if current_head_pos is not None:
            head = list(current_head_pos)
            if len(head) != 2:
                raise ValueError(f"current_head_pos must have length 2, got {len(head)}")
            self._set_scalar_joint(q, "head_yaw_joint", head[0])
            self._set_scalar_joint(q, "head_pitch_joint", head[1])
        return self._clip_q(q)

    def arm_pos_from_q(self, q: np.ndarray) -> list[float]:
        values = []
        for joint_name in ARM_POS_ORDER:
            jid = self.model.getJointId(joint_name)
            values.append(float(q[self.model.idx_qs[jid]]))
        return values

    def ready_arm_pos(self) -> list[float]:
        return list(self.config.ready_arm_pos())

    def joint_limits_for_arm_pos(self) -> list[tuple[str, float, float]]:
        limits = []
        for joint_name in ARM_POS_ORDER:
            jid = self.model.getJointId(joint_name)
            qidx = self.model.idx_qs[jid]
            limits.append(
                (
                    joint_name,
                    float(self.model.lowerPositionLimit[qidx]),
                    float(self.model.upperPositionLimit[qidx]),
                )
            )
        return limits

    def _seed_q(
        self,
        current_arm_pos: Iterable[float] | None,
        current_head_pos: Iterable[float] | None,
        q_seed: np.ndarray | None,
    ) -> np.ndarray:
        if q_seed is not None:
            return self._clip_q(np.asarray(q_seed, dtype=float).copy())
        if current_arm_pos is None:
            current_arm_pos = self.ready_arm_pos()
        return self.q_from_arm_pos(current_arm_pos, current_head_pos)

    def _damped_least_squares(
        self,
        jacobian: np.ndarray,
        err: np.ndarray,
        active_v_idxs: list[int],
    ) -> np.ndarray:
        active_jacobian = jacobian[:, active_v_idxs]
        damping = self.config.damping
        active_velocity = active_jacobian.T @ np.linalg.solve(
            active_jacobian @ active_jacobian.T + damping * np.eye(active_jacobian.shape[0]),
            err,
        )
        velocity = np.zeros(self.model.nv)
        velocity[active_v_idxs] = active_velocity
        return velocity

    def _active_velocity_indices(self, side: ArmSide) -> list[int]:
        idxs = []
        for joint_name in self.config.active_joints_for_side(side):
            jid = self.model.getJointId(joint_name)
            idxs.append(self.model.idx_vs[jid])
        return idxs

    def _clip_q(self, q: np.ndarray) -> np.ndarray:
        margin = self.config.joint_margin
        lower = self.model.lowerPositionLimit + margin
        upper = self.model.upperPositionLimit - margin
        return np.minimum(np.maximum(q, lower), upper)

    def _set_scalar_joint(self, q: np.ndarray, joint_name: str, value: float) -> None:
        if not self.model.existJointName(joint_name):
            raise ValueError(f"URDF is missing joint: {joint_name}")
        jid = self.model.getJointId(joint_name)
        if self.model.joints[jid].nq != 1:
            raise ValueError(f"Joint {joint_name} is not scalar")
        q[self.model.idx_qs[jid]] = float(value)

    def _validate_model(self) -> None:
        for joint_name in ARM_POS_ORDER:
            if not self.model.existJointName(joint_name):
                raise ValueError(f"URDF is missing expected arm joint: {joint_name}")
        for frame_name in [self.config.left_ee_frame, self.config.right_ee_frame]:
            if not self.model.existFrame(frame_name):
                raise ValueError(f"URDF is missing expected end-effector frame: {frame_name}")
        for joint_name in LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS:
            jid = self.model.getJointId(joint_name)
            if self.model.joints[jid].nv != 1:
                raise ValueError(f"Expected scalar joint, got {joint_name}")
