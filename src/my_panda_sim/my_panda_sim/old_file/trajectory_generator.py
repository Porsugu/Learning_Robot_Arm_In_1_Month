"""Day15: Trajectory Generator + IK Solver V2 (PyBullet wrapper, orientation-aware)"""

import pybullet as p
import pybullet_data
import numpy as np
import time

from fk_solver import FKSolver
from ik_solver_v2 import IKSolverV2


class TrajectoryGenerator:
    def __init__(self, dof: int):
        self.dof = dof

    def linear_interpolation(self, q_start, q_goal, steps: int):
        q_start = np.array(q_start)
        q_goal = np.array(q_goal)
        traj = np.linspace(q_start, q_goal, steps)  # shape = (steps, dof)
        return traj