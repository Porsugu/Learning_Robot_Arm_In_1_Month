"""Day18-19: Gripper module for Panda arm"""

import pybullet as p
import time


class Gripper:
    def __init__(self, robot_id, finger_joint_indices, ee_link_index, mode="physics"):
        """
        Panda Gripper control.

        Args:
            robot_id (int): Panda robot unique id in PyBullet
            finger_joint_indices (list[int]): Joint indices for the fingers (usually [9, 10])
            ee_link_index (int): End-effector link index (usually 11)
            mode (str): "teleport" (attach with constraint) or "physics" (realistic friction)
        """
        self.robot_id = robot_id
        self.fingers = finger_joint_indices
        self.ee_link_index = ee_link_index
        self.grasp_constraint = None
        self.mode = mode

        # Ensure fingers have enough friction
        for j in self.fingers:
            p.changeDynamics(
                self.robot_id, j,
                lateralFriction=10.0,
                rollingFriction=0.001,
                spinningFriction=0.001
            )

    def open(self, width=0.08):
        """Open the gripper to the specified width and release object if attached."""
        target = width / 2.0
        for j in self.fingers:
            p.setJointMotorControl2(
                self.robot_id, j, p.POSITION_CONTROL,
                targetPosition=target, force=100
            )
        if self.grasp_constraint is not None:
            p.removeConstraint(self.grasp_constraint)
            self.grasp_constraint = None
            print("[Gripper] Released object.")

    def close(self, obj_id=None, width=None):
        """
        Close the gripper to a specific width.
        If obj_id is provided, either attach with constraint ("teleport")
        or rely on friction ("physics").

        Args:
            obj_id (int, optional): PyBullet body id of the object to grasp
            width (float, optional): Desired opening width (m).
                                     If None → fully close (0.0).
        """
        target = 0.0 if width is None else width / 2.0
        for j in self.fingers:
            p.setJointMotorControl2(
                self.robot_id, j, p.POSITION_CONTROL,
                targetPosition=target, force=150
            )

        if self.mode == "teleport" and obj_id is not None and self.grasp_constraint is None:
            # Stable grasp via constraint
            self.grasp_constraint = p.createConstraint(
                parentBodyUniqueId=self.robot_id,
                parentLinkIndex=self.ee_link_index,
                childBodyUniqueId=obj_id,
                childLinkIndex=-1,
                jointType=p.JOINT_FIXED,
                jointAxis=[0, 0, 0],
                parentFramePosition=[0, 0, 0],
                childFramePosition=[0, 0, 0]
            )
            print("[Gripper] Object attached with constraint.")

        elif self.mode == "physics":
            if obj_id is not None:
                p.changeDynamics(
                    obj_id, -1,
                    lateralFriction=5.0,
                    rollingFriction=0.001,
                    spinningFriction=0.001
                )
            print("[Gripper] Closing in physics mode (no constraint).")

    def set_width(self, width):
        """Set the gripper opening width (does not affect constraints)."""
        target = width / 2.0
        for j in self.fingers:
            p.setJointMotorControl2(
                self.robot_id, j, p.POSITION_CONTROL,
                targetPosition=target, force=100
            )

    def hold(self, duration=0.5):
        """Hold the current state for a specified duration."""
        for _ in range(int(duration / (1.0 / 240.0))):
            p.stepSimulation()
            time.sleep(1.0 / 240.0)

    def wait_until_settled(self, timeout=0.6):
        """Step physics for a short while so the gripper can settle."""
        t_end = time.time() + float(timeout)
        while time.time() < t_end:
            p.stepSimulation()
            time.sleep(1.0 / 240.0)

    def has_contact_with(self, obj_id, min_contacts=1):
        """
        Return True if any finger link has contact with the object.
        NOTE: assumes finger link indices are same as finger joint indices (true for Panda).
        """
        if obj_id is None:
            return False
        cnt = 0
        for link_idx in self.fingers:
            contacts = p.getContactPoints(
                bodyA=self.robot_id, bodyB=obj_id, linkIndexA=link_idx
            )
            if contacts:
                cnt += 1
        return cnt >= int(min_contacts)
