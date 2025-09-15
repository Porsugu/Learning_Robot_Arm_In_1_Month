"""Day17: CartesianTrajectoryExecutor_v2
"""

import time
import numpy as np
import pybullet as p
import pybullet_data

from ik_solver_v2 import IKSolverV2


class CartesianTrajectoryExecutorV2:
    def __init__(self, robot_id, ee_link_index, joint_indices, dt=0.01):
        self.robot_id = robot_id
        self.ee_link_index = ee_link_index
        self.joint_indices = joint_indices
        self.dt = dt
        self.solver = IKSolverV2(robot_id, ee_link_index, dof=len(joint_indices))

    def _get_q(self):
        return np.array([p.getJointState(self.robot_id, j)[0] for j in self.joint_indices], float)

    def _set_q(self, q):
        for j, val in zip(self.joint_indices, q):
            p.setJointMotorControl2(
                bodyIndex=self.robot_id,
                jointIndex=j,
                controlMode=p.POSITION_CONTROL,
                targetPosition=float(val),
                force=200
            )

    def execute_cartesian(self, target_pos, steps=100, down=True, debug=False):
        ls = p.getLinkState(self.robot_id, self.ee_link_index, computeForwardKinematics=True)
        start_pos = np.array(ls[4])
        target_pos = np.array(target_pos, float)

        waypoints = np.linspace(start_pos, target_pos, steps)

        q_current = self._get_q()

        for i, pos in enumerate(waypoints):
            if i < steps - 1:
                q_sol, _ = self.solver.solve(q_current, pos, target_quat=None, down=False)

            else:
                q_sol, _ = self.solver.solve(q_current, pos, target_quat=None, down=down)

            q_current = q_sol
            self._set_q(q_current)
            p.stepSimulation()
            time.sleep(self.dt)

        if debug:
            ls = p.getLinkState(self.robot_id, self.ee_link_index, computeForwardKinematics=True)
            final_pos = np.array(ls[4])
            pos_err = final_pos - target_pos
            print("[ExecutorV2] Target pos:", np.round(target_pos, 4))
            print("[ExecutorV2] Final pos: ", np.round(final_pos, 4))
            print("[ExecutorV2] Error:     ", np.round(pos_err, 6), f"(norm={np.linalg.norm(pos_err):.6f} m)")

