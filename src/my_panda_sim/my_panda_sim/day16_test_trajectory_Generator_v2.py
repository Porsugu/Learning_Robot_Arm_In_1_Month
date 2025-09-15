"""day16: tester"""
import time
import numpy as np
import pybullet as p
import pybullet_data

from fk_solver import FKSolver
from ik_solver_v2 import IKSolverV2
from ik_solver_v3 import IKSolverV3
from Trajectory_Generator_v2 import TrajectoryGenerator


def main():
    # --- PyBullet setup ---
    client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    # Load Panda URDF
    robot_id = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

    dof = 7
    ee_link_index = 11  # panda_hand


    fk = FKSolver(robot_id, list(range(dof)), ee_link_index)
    # ik = IKSolverV2(robot_id, ee_link_index, dof=dof)
    ik = IKSolverV3(robot_id, ee_link_index=11, dof=7, wrist_joint_index=9)
    traj_gen = TrajectoryGenerator(n_steps=50)


    q_start = [0, 0, 0, -1.57, 0, 1.57, 0]  # relaxed home-like pose
    target_pos = [5.45983672e-01, 1.97734416e-12, 5.19242287e-02]
    target_quat = p.getQuaternionFromEuler([np.pi, 0, 0])  # facing down


    joint_traj = traj_gen.cartesian_to_joint_traj(
        ik_solver=ik,
        q_start=q_start,
        goal_pos=target_pos,
        goal_quat=target_quat,
        down=True
    )


    joint_indices = list(range(dof))
    for q in joint_traj:
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
