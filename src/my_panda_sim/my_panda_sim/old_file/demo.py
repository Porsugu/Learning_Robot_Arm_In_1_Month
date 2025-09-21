import pybullet as p
import pybullet_data
import time
import numpy as np

from trajectory_generator_v3 import TrajectoryGeneratorV3
from ik_solver_v2 import IKSolverV2
from executor_v4 import ExecutorV4
from gripper import Gripper


def main():
    # =============================
    # 初始化 PyBullet 環境
    # =============================
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    # 載入場景
    p.loadURDF("plane.urdf")
    robot_id = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)
    cube_id = p.loadURDF("cube_small.urdf", [0.5, 0.0, 0.02])

    ee_link_index = 11
    dof = 7


    ik_solver = IKSolverV2(robot_id, ee_link_index, dof=dof)
    traj_gen = TrajectoryGeneratorV3(steps=2)
    gripper = Gripper(robot_id, [9, 10], ee_link_index=ee_link_index)
    executor = ExecutorV4(robot_id, ee_link_index, ik_solver, traj_gen, dof=dof, gripper=gripper)

    q_home = np.array(ik_solver.home_pose, dtype=float)
    executor._move_joints_smoothly(np.zeros(dof, dtype=float), q_home)
    q_current = q_home

    print("\n===== [Demo] Gripping cube =====")
    q_traj_grip, _ = executor.execute(
        q_init=q_current,
        goal_pos=[0.5, 0.0, 0.02],   # cube 中心位置
        move="grip",
        obj_id=cube_id,
        plot=False,
        print_diff=False,
        down=True
    )
    q_current = np.array(q_traj_grip[-1], dtype=float)

    print("\n===== [Demo] Placing cube =====")
    q_traj_place, _ = executor.execute(
        q_init=q_current,
        goal_pos=[0.2, -0.3, 0.25],
        move="place",
        plot=False,
        print_diff=False,
        down=True
    )
    q_current = np.array(q_traj_place[-1], dtype=float)

    print("\n===== [Demo Finished] =====")
    while True:
        p.stepSimulation()
        time.sleep(1.0 / 240.0)


if __name__ == "__main__":
    main()
