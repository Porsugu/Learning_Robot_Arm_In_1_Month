"""Day15: Inverse Kinematics Solver v2 (PyBullet IK wrapper, orientation-aware, with 'down' option + null space support)"""
"""Day17: Inverse Kinematics Solver v2 (extended with preSolve functionality)
PyBullet IK wrapper with:
- Orientation-aware target handling (supports 'down' option)
- Null space support via joint limits/ranges/rest poses
- preSolve(): returns a home pose aligned with target azimuth when outside field of view,
  making base alignment more robust and reducing IK failures.
"""

import pybullet as p
import numpy as np


class IKSolverV2:
    def __init__(self, body_id, ee_link_index, dof=7, max_iters=500, tol=1e-5, T_w_base=None):
        """
        Initialize the IK solver with robot parameters.
        """
        self.body_id = body_id
        self.ee_link_index = ee_link_index
        self.dof = dof
        self.max_iters = max_iters
        self.tol = tol

        # Joint limits for Franka Panda
        self.lower_limits = [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]
        self.upper_limits = [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973]
        self.joint_ranges = [u - l for l, u in zip(self.lower_limits, self.upper_limits)]

        # A stable home configuration
        # self.home_pose = [0, -np.pi / 4, 0, -np.pi / 2, 0, np.pi / 2, 0]
        self.home_pose = [0, 1 / 4, 0, -np.pi / 3, 0, np.pi / 2, 0]

        # World-to-base transform (optional)
        self.T_w_base = T_w_base if T_w_base is not None else np.eye(4)

    # def preSolve(self, q_init, target_pos, margin=np.pi / 6):
    #     """
    #     Pre-alignment step before IK:
    #     - Checks if target azimuth lies within joint0 ± margin.
    #     - If outside, returns a home pose aligned with target azimuth.
    #     - This reduces singularities and improves IK convergence.
    #     """
    #     q_init = np.array(q_init, dtype=float).copy()
    #
    #     q0 = q_init[0]
    #     target_theta = np.arctan2(target_pos[1], target_pos[0])
    #
    #     diff = np.arctan2(np.sin(target_theta - q0), np.cos(target_theta - q0))
    #     aligned = abs(diff) < margin
    #
    #     if aligned:
    #         return True, q_init
    #
    #     new_q_init = np.array(self.home_pose, dtype=float)
    #     new_q_init[0] = target_theta
    #     return False, new_q_init


    def preSolve(self, q_init, target_pos):

        target_theta = np.arctan2(target_pos[1], target_pos[0])


        new_q_init = np.array(self.home_pose, dtype=float)

        new_q_init[0] = target_theta

        return False, new_q_init

    def solve(self, q_init, target_pos, target_quat=None, down=False):
        """
        Compute IK solution for a target position and orientation.
        - If down=True, enforces tool pointing down.
        - If target_quat=None, keeps current EE orientation.
        - Returns joint configuration close to q_init.
        """
        if down:
            target_quat = p.getQuaternionFromEuler([0, np.pi, 0])

        if target_quat is None:
            ls = p.getLinkState(self.body_id, self.ee_link_index, computeForwardKinematics=True)
            target_quat = ls[5]

        if q_init is None:
            q_init = [0.0] * self.dof

        joint_damping = [0.05] * 11
        result = p.calculateInverseKinematics(
            bodyUniqueId=self.body_id,
            endEffectorLinkIndex=self.ee_link_index,
            targetPosition=target_pos,
            targetOrientation=target_quat,
            lowerLimits=self.lower_limits,
            upperLimits=self.upper_limits,
            jointRanges=self.joint_ranges,
            restPoses=q_init,
            maxNumIterations=self.max_iters,
            residualThreshold=self.tol,
            jointDamping=joint_damping,
        )

        q_sol = np.array(result[:self.dof])

        # Normalize angles to be close to initial guess
        if q_init is not None:
            alt = q_sol + 2 * np.pi * np.round((np.array(q_init) - q_sol) / (2 * np.pi))
            q_sol = alt

        return q_sol, 0


    def _wrap_to_pi(self, a):
        """
        Normalize an angle to the range [-pi, pi].
        """
        return float(np.arctan2(np.sin(a), np.cos(a)))

    def _target_theta_base(self, target_pos):
        """
        Compute target azimuth in base frame XY plane.
        Uses either provided T_w_base or current PyBullet base pose.
        """
        if hasattr(self, "T_w_base") and self.T_w_base is not None:
            T = self.T_w_base
            R = T[:3, :3]
            t = T[:3, 3]
            Pw = np.asarray(target_pos, dtype=float)
            Pb = R.T @ (Pw - t)
            return float(np.arctan2(Pb[1], Pb[0]))

        base_pos, base_quat = p.getBasePositionAndOrientation(self.body_id)
        Rwb = np.array(p.getMatrixFromQuaternion(base_quat)).reshape(3, 3)
        Pw = np.asarray(target_pos, dtype=float)
        Pb = Rwb.T @ (Pw - np.asarray(base_pos))
        return float(np.arctan2(Pb[1], Pb[0]))

    def check_target_theta(self, q0_init, target_pos, margin=np.pi / 2):
        """
        Check if target lies within joint0 ± margin field of view.
        Returns (aligned, target_theta, diff).
        """
        target_theta = self._target_theta_base(target_pos)
        diff = self._wrap_to_pi(target_theta - float(q0_init))
        aligned = abs(diff) <= float(margin)
        return aligned, target_theta, diff
