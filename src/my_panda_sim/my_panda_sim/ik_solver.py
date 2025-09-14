"""day 12- 14: Inverse Kinematics Solver (Jacobian Damped Least Squares)"""
import numpy as np
import pybullet as p
import pybullet_data
import math

from fk_solver import FKSolver, bullet_state_guard


class IKSolver:
    def __init__(self, body_id, joint_indices, ee_link_index,
                 damping=0.05, max_iters=500, tol=1e-3, verbose=False):
        self.body_id = body_id
        self.joint_indices = joint_indices
        self.ee_link_index = ee_link_index
        self.damping = damping
        self.max_iters = max_iters
        self.tol = tol
        self.verbose = verbose
        self.fk = FKSolver(body_id, joint_indices, ee_link_index)

        # Franka Panda joint limits (7 DOF arm)
        self.joint_limits = [
            (-2.8973, 2.8973),
            (-1.7628, 1.7628),
            (-2.8973, 2.8973),
            (-3.0718, -0.0698),
            (-2.8973, 2.8973),
            (-0.0175, 3.7525),
            (-2.8973, 2.8973)
        ]

    def solve(self, q_init, target_pos):
        """Single init"""
        q = np.array(q_init, dtype=float)
        # collect movable joints
        movable_joints = []
        n_joints = p.getNumJoints(self.body_id)
        for j in range(n_joints):
            joint_type = p.getJointInfo(self.body_id, j)[2]
            if joint_type in [p.JOINT_REVOLUTE, p.JOINT_PRISMATIC]:
                movable_joints.append(j)
        dof = len(movable_joints)

        for it in range(self.max_iters):
            pos, _ = self.fk.forward(q)
            err = target_pos - pos
            if self.verbose:
                print(f"iter {it}, error={np.linalg.norm(err):.4f}")

            if np.linalg.norm(err) < self.tol:
                return q, 0  #success

            # build full-length vector
            q_full = [0.0] * dof
            for i, j in enumerate(self.joint_indices):
                if j in movable_joints:
                    q_full[movable_joints.index(j)] = q[i]

            result = p.calculateJacobian(
                self.body_id,
                self.ee_link_index,
                [0, 0, 0],
                q_full,
                [0.0] * dof,
                [0.0] * dof
            )
            if len(result) == 2:
                J_lin, J_ang = result
            else:
                J_lin, J_ang, _ = result

            arm_cols = [movable_joints.index(j) for j in self.joint_indices]
            J_pos = np.array(J_lin)[:, arm_cols]

            JJt = J_pos @ J_pos.T
            lamI = (self.damping ** 2) * np.eye(JJt.shape[0])
            dq = J_pos.T @ np.linalg.inv(JJt + lamI) @ err

            q += dq

            # apply joint limits
            for i in range(len(q)):
                low, high = self.joint_limits[i]
                q[i] = np.clip(q[i], low, high)

        return q, -1  # fail

    def solve_smart_multi(self, q_now, target_pos, trials=10, noise_scale=0.2):
        candidates = []

        # ------
        q_sol, code = self.solve(q_now, target_pos)
        pos_fk, _ = self.fk.forward(q_sol)
        err = np.linalg.norm(target_pos - pos_fk)
        candidates.append((q_sol, code, err, np.linalg.norm(q_sol - q_now)))

        feasible = [(q, c, e, d) for (q, c, e, d) in candidates if c == 0]
        if len(feasible) == 0:
            return None, -1, None

        feasible.sort(key=lambda x: (x[2], x[3]))
        best_q, best_code, best_err, best_dist = feasible[0]
        return best_q, best_code, best_err


# ---------------- test ----------------
if __name__ == "__main__":
    # Setup PyBullet
    p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    robot_id = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

    joint_indices = list(range(7))   # Panda arm 7 DOF
    ee_link_index = 11               # panda_hand link

    ik = IKSolver(robot_id, joint_indices, ee_link_index, verbose=False)

    q_now = [0, 0, 0, 0, 0, 0, 0]
    # q_now = [0, -0.3, 0, -2.0, 0, 2.0, 0.8]
    # target_pos = np.array([0.0, 0.0, 0.0])
    # target_pos = np.array([0.16322435, 0.215286, 0.52320021])
    target_pos = np.array([ 5.46059966e-01, -1.37935843e-12,  5.12618423e-02])
    q_best, code, err = ik.solve_smart_multi(q_now, target_pos, trials=10)
    # q_best, code, err = ik.solve(q_now, target_pos, trials=10)

    print("Smart multi IK return:", code)
    print("Best q:", q_best)
    print("Error:", err)
