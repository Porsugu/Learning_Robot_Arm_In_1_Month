"""Day17: Test IKSolverV2 with trajectory (red=target, blue=final, animate trajectory)"""

import pybullet as p
import pybullet_data
import time
import numpy as np

from ik_solver_v2 import IKSolverV2


def add_debug_sphere(pos, radius=0.03, color=[1, 0, 0, 1]):
    """在 GUI 中畫一顆小球"""
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
    # ---------------- PyBullet 初始化 ----------------
    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    # 載入 Panda
    robot_id = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

    ee_link_index = 11
    joint_indices = [0, 1, 2, 3, 4, 5, 6]

    # 初始化 solver
    solver = IKSolverV2(robot_id, ee_link_index)

    # ---------------- 測試目標 ----------------
    target_pos = [0.4, 0.2, 0.5]
    q_start = [0] * 7

    print("\n[TEST] Solving IK with trajectory ...")
    q_final, trajectory = solver.solve_with_trajectory(
        q_init=q_start,
        target_pos=target_pos,
        down=True,
        debug=True
    )

    print("Final q:", np.round(q_final, 3))
    print("Trajectory length:", len(trajectory))

    # ---------------- 驗證最終位置 ----------------
    for j, val in zip(joint_indices, q_final):
        p.resetJointState(robot_id, j, float(val))

    ls = p.getLinkState(robot_id, ee_link_index, computeForwardKinematics=True)
    final_pos = np.array(ls[4])

    pos_err = final_pos - np.array(target_pos)
    print("Target pos:", np.round(target_pos, 4))
    print("Final pos: ", np.round(final_pos, 4))
    print("Error:     ", np.round(pos_err, 6), f"(norm={np.linalg.norm(pos_err):.6f} m)")

    # ---------------- Debug 球 ----------------
    add_debug_sphere(target_pos, radius=0.03, color=[1, 0, 0, 1])  # target = red
    add_debug_sphere(final_pos, radius=0.03, color=[0, 0, 1, 1])   # final = blue

    # ---------------- 播放軌跡 ----------------
    for q in trajectory:
        for j, val in zip(joint_indices, q):
            p.setJointMotorControl2(
                bodyIndex=robot_id,
                jointIndex=j,
                controlMode=p.POSITION_CONTROL,
                targetPosition=float(val),
                force=200
            )
        p.stepSimulation()
        time.sleep(0.01)

    print("[TEST] Trajectory execution finished. Close GUI to exit.")
    while True:
        p.stepSimulation()
        time.sleep(1.0 / 240.0)


if __name__ == "__main__":
    main()
