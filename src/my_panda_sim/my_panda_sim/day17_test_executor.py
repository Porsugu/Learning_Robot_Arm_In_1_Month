# main_day17.py
import pybullet as p
import pybullet_data
import numpy as np
import time

from Trajectory_Generator_v2 import TrajectoryGenerator
from ik_solver_v2 import IKSolverV2
from executor import Executor


def main():
    # --- 連接 PyBullet ---
    client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    # --- 載入 Panda 機械臂 ---
    robot_id = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)
    num_joints = p.getNumJoints(robot_id)

    # --- 找出可控關節 (7 DOF) ---
    joint_indices = [
        i for i in range(num_joints) if p.getJointInfo(robot_id, i)[2] == p.JOINT_REVOLUTE
    ]

    # --- IK Solver (V3: 支援 wrist joints + link 垂直對齊) ---
    ik_solver = IKSolverV2(
        body_id=robot_id,
        ee_link_index=11,          # panda_hand
        dof=len(joint_indices),
    )

    # --- Trajectory Generator (檢查垂直性) ---
    traj_gen = TrajectoryGenerator(
        n_steps=100
    )

    # --- 初始關節狀態 ---
    q_start = [p.getJointState(robot_id, i)[0] for i in joint_indices]

    # --- 設定目標末端位姿 ---
    target_pos = [0.55, 0.0, 0.5]
    target_orn = p.getQuaternionFromEuler([np.pi, 0, 0])  # 嘗試手掌朝下

    # --- 產生 Joint Trajectory ---
    joint_traj = traj_gen.cartesian_to_joint_traj(
        ik_solver=ik_solver,
        q_start=q_start,
        goal_pos=target_pos,
        goal_quat=target_orn,
        down=True
    )

    # --- 執行軌跡 ---
    executor = Executor(robot_id, joint_indices, dt=0.01)
    executor.execute_trajectory(joint_traj)

    # 保持視窗開著
    while True:
        time.sleep(0.01)


if __name__ == "__main__":
    main()
