# Week 2 | Forward & Inverse Kinematics

## 📆 Scope
Week 2 covers **Day 8 – Day 14** of the 30-day plan.  
The focus is on building modular tools for **Forward Kinematics (FK)** and stepping into **Inverse Kinematics (IK)**.

---

## Overview of the Week
- **Day 8**: Modular Forward Kinematics tool (class-based design)
- **Day 9**: Matrix chain implementation of FK
- **Day 10**: Comparison: PyBullet FK vs hand-written FK
- **Day 11**: Rotation matrices and Euler angle conversions
- **Day 12**: Deeper practice with homogeneous transformations
- **Day 13**: Introduction to numerical IK
- **Day 14**: Testing IK with constraints



## Week 2 Learning Reflection (Day8–Day14)

During this week I focused on building both the implementation and intuition of forward and inverse kinematics.

- **Forward Kinematics (FK):**  
  I implemented a modular FK solver that takes joint angles \(q\) and computes the end-effector pose with respect to the base frame. By chaining homogeneous transformations:

  $$
  T_{0}^{ee}(q) = \prod_{i=1}^{n} T_{i-1}^{i}(q_i)
  $$

  I understood why FK is deterministic and always produces a unique solution given the configuration.

- **Inverse Kinematics (IK):**  
  I implemented a Jacobian-based solver using Damped Least Squares (DLS). The Jacobian \(J\) links joint velocities \(\dot{q}\) to end-effector velocities \(\dot{x}\):

  $$
  \dot{x} = J(q)\,\dot{q}
  $$

  To iteratively reduce the position error \(e = x^* - x(q)\), I applied the DLS update rule:

  $$
  \Delta q = J^{	op}\left(JJ^{	op} + \lambda^{2} I
ight)^{-1} e
  $$

  This taught me how damping \(\lambda\) ensures numerical stability near singularities by making the system well-conditioned.

- **Practical Intuition:**  
  I learned that IK may not always converge: results depend on initial guesses, joint limits, and redundancy. Multiple initializations or null-space objectives can increase success rates. Compared to FK, which is direct and exact, IK is iterative and approximate.

Although I still need deeper study on matrix calculus to rigorously derive these results, I can now confidently explain the principles behind FK and IK and understand why in real engineering tasks, robust solvers like **TRAC-IK** or **MoveIt** are often preferred when handling complex grasping or orientation constraints.





## Day8

### Achievement

Built a Forward Kinematics tool class that converts joint angles into end-effector position and orientation.
Designed in a modular way so it can be reused in later days (matrix FK, IK, motion planning).
Successfully tested with the Franka Panda robot in PyBullet.

### How to Run
Navigate to your package directory:  
   ```
   bash
   cd ~/ros2_ws/src/my_panda_sim/my_panda_sim
   python3 test_fk.py
   ```
---
## Day9
### Achievement
Built a MatrixChainDebugger class to print each step and the cumulative result
Verified matrix multiplication order with a test script

### How to Run
```
   cd ~/ros2_ws/src/my_panda_sim/my_panda_sim
   python3 test_matrix_chain.py

```
---
## Day10
### Achievement
Implemented ForwardKinematics (FKSolver) as a reusable class that can compute the end-effector pose from given joint angles.

Built a ROS2 node (fk_publisher) that:

Uses FKSolver internally.

Publishes the computed end-effector pose to the topic /fk_pose.

Currently uses a fixed test joint configuration (q_test) for demonstration.

Created a launch file (display.launch.py) to start:

robot_state_publisher with Panda URDF.

A static transform (world → panda_link0).

The fk_publisher node.

Integrated everything into setup.py with proper console entry points.

Outcome: The FK tool is no longer just a math script — it is now a proper ROS2 node that can publish results, preparing the system for multi-node integration in later weeks.

---
## Day11
### Achievement

Upgraded ForwardKinematics into a modular FKSolver:
Added bullet_state_guard to protect simulation state.
Supports arbitrary target link index, not limited to end-effector.
Provides two output modes:

(pos, quat) → for ROS2 PoseStamped
T_base_link (4×4) → for Jacobian, IK, and trajectory planning

Implemented FKPublisher ROS2 node:
Subscribes to /joint_states for joint angles.
Uses FKSolver to compute forward kinematics.
Publishes /fk_pose (PoseStamped) with the end-effector pose in the base frame.
Achieved a complete pipeline: JointState → FK → PoseStamped → ROS2 topic.

---
## Day 12-14
### Achievement
Implemented a Jacobian-based Damped Least Squares (DLS) IK solver, capable of solving end-effector positions from arbitrary joint configurations.
Gained practical understanding of how Jacobian relates joint velocities to end-effector motions, and how damping stabilizes solutions near singularities.
Explored joint limits, convergence issues, and developed intuition about when IK succeeds, fails, or requires multiple initial guesses.

---


