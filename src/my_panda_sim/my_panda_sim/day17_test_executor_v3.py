import pybullet as p
import pybullet_data
import numpy as np
import time

from trajectory_generator_v3 import TrajectoryGeneratorV3
from executor_v3 import ExecutorV3
from ik_solver_v2 import IKSolverV2


def main():
    # init environment
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    robot_id = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)
    ee_link_index = 11  # Panda hand
    dof = 7

    # Init toold
    ik_solver = IKSolverV2(robot_id, ee_link_index, dof=dof)
    traj_gen = TrajectoryGeneratorV3(steps=100)
    executor = ExecutorV3(robot_id, ee_link_index, ik_solver, traj_gen, dof=dof)

    # Move to home pose
    print("Moving to initial home_pose...")
    q_start_pose = np.array(ik_solver.home_pose)
    executor._move_joints_smoothly(np.array([0] * dof), q_start_pose)
    print("Initial position reached.")
    time.sleep(1)

    waypoints = [
        [0.5, -0.2, 0.5],
        [0.5, 0.2, 0.5],
        [0.5, 0.2, 0.2],
        [0.5, -0.2, 0.2],
        [0.5, -0.2, 0.5]
    ]

    # make balls for target
    for point in waypoints:
        sphere = p.createVisualShape(p.GEOM_SPHERE, radius=0.05, rgbaColor=[0, 1, 0, 0.8])
        p.createMultiBody(baseVisualShapeIndex=sphere, basePosition=point)

    # Test more
    current_q = q_start_pose

    for i, goal_pos in enumerate(waypoints):
        print(f"\n===== Moving to waypoint {i + 1}: {goal_pos} =====")

        q_traj, _ = executor.execute(current_q, goal_pos, down=True, plot=False, print_diff=False)

        current_q = np.array(q_traj[-1])

        print(f"Waypoint {i + 1} reached. Pausing for 1 second.")
        time.sleep(1)

    print("\nAll waypoints have been visited. Mission complete.")

    executor._freeze_kinematic(robot_id, dof, current_q)
    while True:
        p.stepSimulation()
        time.sleep(1. / 240.)

    # p.disconnect() #


if __name__ == "__main__":
    main()