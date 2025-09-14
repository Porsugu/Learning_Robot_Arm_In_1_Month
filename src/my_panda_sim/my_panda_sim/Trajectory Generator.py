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


def main():

    client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    robot_id = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)
    ls = p.getLinkState(robot_id, 11, computeForwardKinematics=True)
    pos, orn = ls[4], ls[5]
    rotm = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
    print("EE axes (x,y,z):")
    print(rotm)


    dof = 7
    fk = FKSolver(robot_id, list(range(dof)), ee_link_index=11)  # panda_hand = 11
    ik = IKSolverV2(robot_id, ee_link_index=11, dof=dof)
    traj_gen = TrajectoryGenerator(dof=dof)


    q_start = [0, 0, 0, 0, 0, 0, 0]


    target_pos = [0.4, 0.2, 0.3]

    # Grabber to the ground
    target_quat = p.getQuaternionFromEuler([np.pi, 0, 0])
    #IK
    q_goal, code = ik.solve(q_start, target_pos, target_quat, down=True)

    print("Start q:", np.round(q_start, 3))
    print("Goal  q:", np.round(q_goal, 3))

    if code != 0:
        print("IK failed, aborting trajectory.")
        return

    traj = traj_gen.linear_interpolation(q_start, q_goal, steps=100)


    joint_indices = list(range(dof))
    for q in traj:
        for j in range(dof):
            p.setJointMotorControl2(
                robot_id,
                joint_indices[j],
                p.POSITION_CONTROL,
                targetPosition=q[j],
                force=87
            )
        p.stepSimulation()

        pos, quat = fk.forward(q)
        print(f"EE pos: {np.round(pos,3)}, quat: {np.round(quat,3)}")

        time.sleep(0.05)


    print("Trajectory execution finished.")
    time.sleep(2)
    p.disconnect()


if __name__ == "__main__":
    main()
