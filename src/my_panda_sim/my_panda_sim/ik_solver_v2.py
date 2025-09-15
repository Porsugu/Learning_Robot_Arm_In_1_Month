"""Day15: Inverse Kinematics Solver v2 (PyBullet IK wrapper, orientation-aware, with 'down' option)"""

import pybullet as p
import numpy as np

class IKSolverV2:
    def __init__(self, body_id, ee_link_index, dof=7, max_iters=200, tol=1e-4):
        self.body_id = body_id
        self.ee_link_index = ee_link_index
        self.dof = dof
        self.max_iters = max_iters
        self.tol = tol

    def solve(self, q_init, target_pos, target_quat=None, down=False):
        if down:
            target_quat = p.getQuaternionFromEuler([0, np.pi, 0])
        if target_quat is None:
            ls = p.getLinkState(self.body_id, self.ee_link_index, computeForwardKinematics=True)
            target_quat = ls[5]

        result = p.calculateInverseKinematics(
            self.body_id,
            self.ee_link_index,
            target_pos,
            target_quat,
            maxNumIterations=self.max_iters,
            residualThreshold=self.tol
        )
        q_sol = np.array(result[:self.dof])

        if q_init is not None:
            alt = q_sol + 2 * np.pi * np.round((np.array(q_init) - q_sol) / (2 * np.pi))
            q_sol = alt

        return q_sol, 0




