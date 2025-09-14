"""day16: Cartesian interpolation"""
import numpy as np
import pybullet as p
import pybullet_data

from fk_solver import FKSolver
from ik_solver_v2 import IKSolverV2


class TrajectoryGenerator:
    def __init__(self, n_steps=50):
        """
        n_steps: number of interpolation steps
        """
        self.n_steps = n_steps

    def interpolate(self, start_pos, start_quat, goal_pos, goal_quat):
        """
        Cartesian interpolation: linear pos + SLERP quat
        """
        traj = []
        for i in range(self.n_steps + 1):
            alpha = i / self.n_steps
            pos = (1 - alpha) * np.array(start_pos) + alpha * np.array(goal_pos)
            quat = p.getQuaternionSlerp(start_quat, goal_quat, alpha)
            traj.append((pos, quat))
        return traj

    def cartesian_to_joint_traj(self, ik_solver, q_start, goal_pos, goal_quat=None, down=False):
        """
        Convert Cartesian trajectory to joint trajectory via IKSolverV2.
        """
        # Forward from start config
        start_pos, start_quat = p.getLinkState(
            ik_solver.body_id, ik_solver.ee_link_index, computeForwardKinematics=True
        )[4:6]


        cart_traj = self.interpolate(start_pos, start_quat, goal_pos, goal_quat if goal_quat else start_quat)

        joint_traj = []
        q_curr = q_start
        for pos, quat in cart_traj:
            q_curr, _ = ik_solver.solve(q_curr, pos, quat, down=down)
            joint_traj.append(q_curr)
        return joint_traj

