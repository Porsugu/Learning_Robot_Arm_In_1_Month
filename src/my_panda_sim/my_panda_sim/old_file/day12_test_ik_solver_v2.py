"""Day15: Test IKSolverV2 (multi-point test, red=target, blue=final)"""

import pybullet as p
import pybullet_data
import time
import numpy as np

from ik_solver_v2 import IKSolverV2


def add_debug_sphere(pos, radius=0.03, color=[1, 0, 0, 1]):
    visual = p.createVisualShape(
        shapeType=p.GEOM_SPHERE,
        radius=radius,
        rgbaColor=color
    )
    collision = p.createCollisionShape(
        shapeType=p.GEOM_SPHERE,
        radius=radius
    )
    body = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=collision,
        baseVisualShapeIndex=visual,
        basePosition=pos
    )
    return body


def main():
    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    robot_id = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)
    ee_link_index = 11
    joint_indices = [0, 1, 2, 3, 4, 5, 6]

    solver = IKSolverV2(robot_id, ee_link_index)

    target_points = [
        [0.4, 0.2, 0.5],
        [0.3, -0.3, 0.45],
        [0.5, 0.0, 0.35],
    ]

    for i, target_pos in enumerate(target_points):
        print(f"\n[TEST {i+1}] Target = {target_pos}")

        q_start = [0] * 7

        q_sol, _ = solver.solve(q_start, target_pos, down=True)

        for j, val in zip(joint_indices, q_sol):
            p.resetJointState(robot_id, j, float(val))

        ls = p.getLinkState(robot_id, ee_link_index, computeForwardKinematics=True)
        final_pos = np.array(ls[4])

        pos_err = final_pos - np.array(target_pos)
        print("Target pos:", np.round(target_pos, 4))
        print("Final pos: ", np.round(final_pos, 4))
        print("Error:     ", np.round(pos_err, 6),
              f"(norm={np.linalg.norm(pos_err):.6f} m)")

        add_debug_sphere(target_pos, radius=0.03, color=[1, 0, 0, 1])
        add_debug_sphere(final_pos, radius=0.03, color=[0, 0, 1, 1])

        time.sleep(1.0)

    for _ in range(600):
        p.stepSimulation()
        time.sleep(1.0 / 240.0)

    p.disconnect()


if __name__ == "__main__":
    main()
