"""Day17: Test CartesianTrajectoryExecutorV2 (三點測試 + debug 球)"""

import pybullet as p
import pybullet_data
import time
import numpy as np

from ik_solver_v2 import IKSolverV2
from executor_v2 import CartesianTrajectoryExecutorV2


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

    executor = CartesianTrajectoryExecutorV2(robot_id, ee_link_index, joint_indices)

    targets = [
        [0.4, 0.2, 0.5],
        [0.3, -0.3, 0.45],
        [0.5, 0.0, 0.35]
    ]

    for i, target in enumerate(targets, 1):
        print(f"\n[TEST] Moving to target {i}: {target}")

        add_debug_sphere(target, radius=0.03, color=[1, 0, 0, 1])

        executor.execute_cartesian(target, steps=80, down=True, debug=True)

        ls = p.getLinkState(robot_id, ee_link_index, computeForwardKinematics=True)
        final_pos = np.array(ls[4])
        add_debug_sphere(final_pos, radius=0.03, color=[0, 0, 1, 1])

        for _ in range(240):
            p.stepSimulation()
            time.sleep(1.0 / 240.0)

    p.disconnect()


if __name__ == "__main__":
    main()
