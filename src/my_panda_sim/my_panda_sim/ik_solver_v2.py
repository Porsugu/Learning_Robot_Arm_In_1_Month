"""Day15: Inverse Kinematics Solver v2 (PyBullet IK wrapper, orientation-aware, with 'down' option + null space support)"""
import pybullet as p
import numpy as np

class IKSolverV2:
    def __init__(self, body_id, ee_link_index, dof=7, max_iters=500, tol=1e-5):
        self.body_id = body_id
        self.ee_link_index = ee_link_index
        self.dof = dof
        self.max_iters = max_iters
        self.tol = tol

        self.lower_limits = [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]
        self.upper_limits = [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973]

        self.joint_ranges = [u - l for l, u in zip(self.lower_limits, self.upper_limits)]
        self.home_pose = [0, -np.pi / 4, 0, -np.pi / 2, 0, np.pi / 2, 0]

    def solve(self, q_init, target_pos, target_quat=None, down=False):
        if down:
            target_quat = p.getQuaternionFromEuler([0, np.pi, 0])

        if target_quat is None:
            ls = p.getLinkState(self.body_id, self.ee_link_index, computeForwardKinematics=True)
            target_quat = ls[5]

        if q_init is None:
            q_init = [0.0] * self.dof

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
            residualThreshold=self.tol
        )

        q_sol = np.array(result[:self.dof])

        if q_init is not None:
            alt = q_sol + 2 * np.pi * np.round((np.array(q_init) - q_sol) / (2 * np.pi))
            q_sol = alt

        return q_sol, 0
