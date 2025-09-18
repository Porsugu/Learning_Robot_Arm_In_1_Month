"""
Day18: Panda robot executes a trajectory and checks collision with a box.
"""

import pybullet as p
import pybullet_data
import time
import numpy as np
from CollisionTracker import CollisionTracker

def create_box(position, half_extents=(0.05, 0.05, 0.05), rgba=(1,0,0,1), mass=0.0):
    """Create a box in the simulation world."""
    col_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
    vis_id = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents, rgbaColor=rgba)
    return p.createMultiBody(baseMass=mass,
                             baseCollisionShapeIndex=col_id,
                             baseVisualShapeIndex=vis_id,
                             basePosition=position)

def panda_ready_q():
    """Return the default ready joint configuration for the Panda arm."""
    return np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])

def main():
    # Initialize physics client
    cid = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    # Load Panda robot
    robot_id = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

    # Add a red box as an obstacle
    box_id = create_box((0.4, 0, 0.5), rgba=(1, 0, 0, 0.8))

    # Create simple collision tracker
    tracker = CollisionTracker(robot_id, [box_id], safe_distance=0.5)

    # Generate a simple trajectory
    active_joints = list(range(7))
    q_start = panda_ready_q()
    q_goal = q_start + np.array([0.2, -0.1, 0.1, 0.1, 0, -0.1, 0.1])

    steps = 50
    for i in range(steps):
        q = q_start + (q_goal - q_start) * (i / steps)
        for ji, ang in zip(active_joints, q):
            p.resetJointState(robot_id, ji, float(ang))
        p.stepSimulation()

        # Collision check
        if tracker.check():
            print(f"[Step {i}] ⚠️ Collision detected!")
        else:
            print(f"[Step {i}] Safe")

        time.sleep(0.02)

    # Keep GUI open until manually closed
    while p.isConnected():
        time.sleep(0.2)

if __name__ == "__main__":
    main()
