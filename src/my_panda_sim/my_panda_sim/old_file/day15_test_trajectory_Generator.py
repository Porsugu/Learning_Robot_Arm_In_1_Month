import time
import numpy as np
import pybullet as p
import pybullet_data

from fk_solver import FKSolver
from ik_solver_v2 import IKSolverV2
from trajectory_generator import TrajectoryGenerator


def main():
    # --- PyBullet setup ---
    client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.gsetDataPath())
    p.setGravity(0, 0, -9.81)

    # Load Panda URDF
    robot_id = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

    dof = 7
    ee_link_index = 11  # panda_hand

    # --- Init solvers ---
    fk = FKSolver(robot_id, list(range(dof)), ee_link_index)
    ik = IKSolverV2(robot_id, ee_link_index, dof=dof)
    traj_gen = TrajectoryGenerator(dof=dof)

    # --- Start / Goal config ---
    q_start = [0, 0, 0, 0, 0, 0, 0]
    target_pos = [0.4, 0.2, 0.3]
    target_quat = p.getQuaternionFromEuler([np.pi, 0, 0])  # facing down

    # IK goal
    q_goal, code = ik.solve(q_start, target_pos, target_quat, down=True)
    print("Start q:", np.round(q_start, 3))
    print("Goal  q:", np.round(q_goal, 3))

    if code != 0:
        print("IK failed, aborting trajectory.")
        return

    # --- Generate joint interpolation ---
    traj = traj_gen.linear_interpolation(q_start, q_goal, steps=100)

    # --- Execute trajectory ---
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
