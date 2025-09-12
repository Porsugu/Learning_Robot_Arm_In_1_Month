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

---
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
