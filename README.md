# Week 3 | Trajectory & Collision （Day15–Day21）

## 📆 Scope
Week 3 covers Day 15 – Day 21 of the 30-day plan.
The focus is on building modular tools for trajectory generation and execution, while preparing the framework for collision checking and error analysis.

---

## Overview of the Week
- **Day 15**: Introduced TrajectoryGenerator and implemented joint-space trajectory planning combined with the new IKSolverV2 (orientation-aware, palm-down option).
- **Day 16**: Extended TrajectoryGenerator with Cartesian-space interpolation (linear position + quaternion Slerp) and integrated IK for smoother end-effector motion.
- **Day 17**: Designed an Executor module to handle step-by-step trajectory execution in PyBullet, ensuring time-synchronized control of joints.
- **Day 18**: Enhanced the Executor with logging and replay capabilities, laying groundwork for analyzing trajectory performance.
- **Day 19**: Started implementing a CollisionChecker using PyBullet’s built-in collision detection APIs, testing basic self-collision and obstacle interaction.
- **Day 20**: Integrated CollisionChecker with the planner to reject infeasible trajectories, ensuring safe execution paths.
- **Day 21**: Developed an ErrorAnalyzer to compare executed vs. planned trajectories, measuring positional/orientation error and preparing visualization tools.

## Week 3 Learning Reflection （Day15–Day21）
---
## Day15:
Implemented IKSolverV2 supporting orientation-aware inverse kinematics:

Added palm-down (down=True) option for end-effector orientation control

Integrated nearest-solution correction to avoid joint discontinuities

Combined with TrajectoryGenerator for joint-space interpolation

Successfully executed trajectories in PyBullet GUI, moving the Panda arm from home pose to target (0.4, 0.2, 0.3) with the end-effector correctly oriented palm-down

---
## Day16:
Implemented TrajectoryGenerator_v2 supporting Cartesian interpolation:

Position: linear interpolation

Orientation: quaternion SLERP

Integrated with IKSolverV2, enabling conversion from Cartesian waypoints to joint trajectories.

Successfully executed trajectories in PyBullet GUI, visualizing end-effector motion along smooth Cartesian paths.

---
## Day17:
Day17 marks the creation of the Executor module, which closes the loop from trajectory planning → playback on the Panda arm in PyBullet. Unlike earlier stages, this implementation integrates a robust preSolve alignment step: before attempting IK, the solver checks whether the target lies within the arm’s current “field of view.” If not, the system re-aligns by moving the base joint to a home pose facing the target, ensuring stability and avoiding singular configurations.

The execution pipeline is fully modular and proceeds as follows:

Target input (goal position)

preSolve: check azimuth alignment; if outside margin, re-align via base joint adjustment to home pose

IKSolver: compute a valid joint configuration for the target

Executor: smoothly perform the joint motion with POSITION_CONTROL, respecting safe step sizes and timing

Completion: trajectory is executed reliably, even for targets in different orientations

This workflow ensures that the robot can now grasp objects from multiple orientations without falling into unstable IK solutions. By enforcing viewpoint constraints and modularizing each step (TrajectoryGenerator, IKSolver, Executor), the system achieves both robustness and flexibility—components can be swapped or optimized independently.

Day17 establishes the practical foundation for upcoming modules such as CollisionChecker and ErrorAnalyzer, proving that smooth Cartesian → joint-space trajectories can be generated, safeguarded against singularities, and executed in real-time.

---
## Day18
Achievement

Implemented a simple collision tracker for the Panda robot in PyBullet.

Supports checking whether the robot is colliding (or within a safety margin) with obstacles.

Integrated with a demo trajectory executor for real-time safety monitoring.
