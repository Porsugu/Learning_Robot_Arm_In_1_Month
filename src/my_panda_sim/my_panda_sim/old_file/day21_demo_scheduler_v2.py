"""Demo: TaskSchedulerV2 with dynamic task queue"""

import pybullet as p
import pybullet_data
import numpy as np
import time

from trajectory_generator_v3 import TrajectoryGeneratorV3
from ik_solver_v2 import IKSolverV2
from gripper import Gripper
from executor_v4 import ExecutorV4
from task_scheduler_v2 import TaskSchedulerV2


def main():
    # Init sim
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    plane_id = p.loadURDF("plane.urdf")
    robot_id = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

    ee_link_index = 11
    dof = 7
    finger_joints = [9, 10]

    ik_solver = IKSolverV2(robot_id, ee_link_index, dof)
    traj_gen = TrajectoryGeneratorV3(steps=10)
    gripper = Gripper(robot_id, finger_joints, ee_link_index)
    executor = ExecutorV4(robot_id, ee_link_index, ik_solver, traj_gen, dof, gripper=gripper, collision_checker=None)

    # Home pose
    q_home = np.array(ik_solver.home_pose)
    for j in range(dof):
        p.resetJointState(robot_id, j, q_home[j])
    print("[Demo] Robot at home pose.")

    # Fixed drop-off location
    drop_pos = [0.0, 0.5, 0.1]

    # Scheduler
    scheduler = TaskSchedulerV2(executor, ik_solver, drop_pos)

    # Spawn initial boxes
    box_positions = [[0.5, 0.0, 0.02], [0.6, -0.2, 0.02]]
    obj_ids = []
    for pos in box_positions:
        obj_id = p.loadURDF("cube_small.urdf", pos)
        obj_ids.append(obj_id)
        scheduler.add_task(obj_id, pick_pos=pos)

    # Run tasks (with dynamic insertion)
    q_cur = q_home
    for i in range(2):
        q_cur = scheduler.run_next(q_cur)

        # Add task while running
        if i == 0:
            new_obj = p.loadURDF("cube_small.urdf", [0.4, 0.3, 0.02])
            scheduler.add_task(new_obj)


    scheduler.run_all(q_cur)

    # Keep GUI alive
    while True:
        p.stepSimulation()
        time.sleep(1.0 / 240.0)


if __name__ == "__main__":
    main()
