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
