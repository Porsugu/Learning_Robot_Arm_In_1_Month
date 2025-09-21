## Progress Summary
## Week 1 (Day1–Day7) – Foundations

Set up the PyBullet simulation environment with the Franka Panda robot.

Implemented Forward Kinematics (FK) to verify end-effector pose calculations.

Studied Jacobian formulation and singularity issues.

## Week 2 (Day8–Day14) – Inverse Kinematics

Built a Jacobian-based IK solver using Damped Least Squares (DLS).

Enabled control of both position and orientation.

Packaged the IK solver into a reusable module.

## Week 3 (Day15–Day21) – Trajectory, Execution, and Task Scheduling

Trajectory generation: Developed a TrajectoryGenerator for Cartesian interpolation with orientation SLERP, mapped to joint trajectories via IK.

Execution: Added an Executor capable of executing trajectories on the robot in simulation, with safe return-to-home, gripper control, and collision pause/resume.

Gripper & Pick strategy: Implemented a gripper module and a structured pick motion (approach → descend → grasp → lift).

Task scheduling & demo UI:

Built TaskSchedulerV2 with a dynamic task queue, enabling automated workflows: spawn cube → pick → place → home.

Integrated a PyQt5 demo UI with a "Spawn Cube" button; cubes are added to the queue and executed automatically by the robot.
