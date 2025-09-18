import pybullet as p
import pybullet_data
import numpy as np
import time

from trajectory_generator_v3 import TrajectoryGeneratorV3
from executor_v3 import ExecutorV3
from ik_solver_v2 import IKSolverV2


def main():
    # Initialize simulation
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    # Load Panda robot
    robot_id = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)
    ee_link_index = 11
    dof = 7

    # Initialize modules
    ik_solver = IKSolverV2(robot_id, ee_link_index, dof=dof)
    traj_gen = TrajectoryGeneratorV3(steps=2)
    executor = ExecutorV3(robot_id, ee_link_index, ik_solver, traj_gen, dof=dof)

    # Move to home pose
    q_start_pose = np.array(ik_solver.home_pose)
    executor._move_joints_smoothly(np.array([0] * dof), q_start_pose)

    # Define waypoints
    waypoints = [
        [-0.5, -0.4, 0.6],
        [0.6, 0.4, 0.6],
        [0.6, -0.4, 0.3],
        [-0.6, 0.4, 0.3],
        [0.0, 0.0, 0.7],  #

    ]

    # Visualize targets
    for point in waypoints:
        sphere = p.createVisualShape(p.GEOM_SPHERE, radius=0.05, rgbaColor=[0, 1, 0, 0.8])
        p.createMultiBody(baseVisualShapeIndex=sphere, basePosition=point)

    # Execute waypoints
    current_q = q_start_pose
    for i, goal_pos in enumerate(waypoints):
        print(f"\n===== Moving to waypoint {i + 1}: {goal_pos} =====")
        q_traj, _ = executor.execute(current_q, goal_pos, down=True, plot=False, print_diff=False)
        current_q = np.array(q_traj[-1])

        time.sleep(1)

    print("\nAll waypoints completed.")
    executor._freeze_kinematic(robot_id, dof, current_q)

    while True:
        p.stepSimulation()
        time.sleep(1. / 240.)


if __name__ == "__main__":
    main()
