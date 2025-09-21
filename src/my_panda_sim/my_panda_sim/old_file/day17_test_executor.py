"""Day17: Test Executor with IKSolverV2 + TrajectoryGenerator"""

import pybullet as p
import pybullet_data
import time
import numpy as np

from ik_solver_v2 import IKSolverV2
from trajectory_generator import TrajectoryGenerator
from executor import Executor


def add_debug_sphere(pos, radius=0.03, color=[1, 0, 0, 1]):
    """在 GUI 畫一顆小球"""
    visual = p.createVisualShape(
        shapeType=p.GEOM_SPHERE,
        radius=radius,
        rgbaColor=color
    )
    body = p.createMultiBody(
        baseMass=0,
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

    # Joint 配置
    joint_indices = [0, 1, 2, 3, 4, 5, 6]   # Panda 7 DOF
    ee_link_index = 11  # panda_hand

    # 初始化工具
    solver = IKSolverV2(robot_id, ee_link_index)
    traj_gen = TrajectoryGenerator(len(joint_indices))
    executor = Executor(robot_id, joint_indices, dt=0.01)

    # ---------------- 測試目標 ----------------
    target_pos = [0.4, 0.2, 0.5]
    q_start = np.zeros(len(joint_indices))

    # IK 求解
    q_goal, _ = solver.solve(q_start, target_pos, down=True)

    # 生成軌跡
    traj = traj_gen.linear_interpolation(q_start, q_goal, steps=100)

    # 執行軌跡
    executor.execute_trajectory(traj)

    # ---------------- 驗證 ----------------
    ls = p.getLinkState(robot_id, ee_link_index, computeForwardKinematics=True)
    final_pos = np.array(ls[4])

    print("Target pos:", np.round(target_pos, 4))
    print("Final pos: ", np.round(final_pos, 4))
    print("Error:     ", np.round(final_pos - target_pos, 6),
          f"(norm={np.linalg.norm(final_pos - target_pos):.6f} m)")

    # ---------------- Debug 球 ----------------
    add_debug_sphere(target_pos, radius=0.03, color=[1, 0, 0, 1])  # 紅球 = 目標
    add_debug_sphere(final_pos, radius=0.03, color=[0, 0, 1, 1])  # 藍球 = 實際

    # ---------------- 模擬展示 ----------------
    for _ in range(600):
        p.stepSimulation()
        time.sleep(1.0 / 240.0)

    p.disconnect()


if __name__ == "__main__":
    main()
