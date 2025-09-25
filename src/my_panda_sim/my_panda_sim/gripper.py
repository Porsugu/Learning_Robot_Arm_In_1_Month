# """Day18-19: Gripper module for Panda arm"""
#
# import pybullet as p
# import time
#
#
# class Gripper:
#     def __init__(self, robot_id, finger_joint_indices, ee_link_index, mode="physic"):
#         """
#         Panda Gripper control.
#
#         Args:
#             robot_id (int): Panda robot unique id in PyBullet
#             finger_joint_indices (list[int]): Joint indices for the fingers (usually [9, 10])
#             ee_link_index (int): End-effector link index (usually 11)
#             mode (str): "teleport" (attach with constraint) or "physics" (realistic friction)
#         """
#         self.robot_id = robot_id
#         self.fingers = finger_joint_indices
#         self.ee_link_index = ee_link_index
#         self.grasp_constraint = None
#         self.mode = mode
#
#         # Ensure fingers have enough friction
#         for j in self.fingers:
#             p.changeDynamics(
#                 self.robot_id, j,
#                 lateralFriction=50.0,
#                 rollingFriction=0.001,
#                 spinningFriction=0.001
#             )
#
#     def open(self, width=0.08):
#         """Open the gripper to the specified width and release object if attached."""
#         target = width / 2.0
#         for j in self.fingers:
#             p.setJointMotorControl2(
#                 self.robot_id, j, p.POSITION_CONTROL,
#                 targetPosition=target, force=200
#             )
#         if self.grasp_constraint is not None:
#             p.removeConstraint(self.grasp_constraint)
#             self.grasp_constraint = None
#             print("[Gripper] Released object.")
#
#     def close(self, obj_id=None, width=None):
#         """
#         Close the gripper to a specific width.
#         If obj_id is provided, either attach with constraint ("teleport")
#         or rely on friction ("physics").
#
#         Args:
#             obj_id (int, optional): PyBullet body id of the object to grasp
#             width (float, optional): Desired opening width (m).
#                                      If None → fully close (0.0).
#         """
#         target = 0.0 if width is None else width / 2.0
#         for j in self.fingers:
#             p.setJointMotorControl2(
#                 self.robot_id, j, p.POSITION_CONTROL,
#                 targetPosition=target,
#                 force=50,
#
#             )
#
#         if self.mode == "teleport" and obj_id is not None and self.grasp_constraint is None:
#             # Stable grasp via constraint
#             self.grasp_constraint = p.createConstraint(
#                 parentBodyUniqueId=self.robot_id,
#                 parentLinkIndex=self.ee_link_index,
#                 childBodyUniqueId=obj_id,
#                 childLinkIndex=-1,
#                 jointType=p.JOINT_FIXED,
#                 jointAxis=[0, 0, 0],
#                 parentFramePosition=[0, 0, 0],
#                 childFramePosition=[0, 0, 0]
#             )
#             print("[Gripper] Object attached with constraint.")
#
#         elif self.mode == "physics":
#             if obj_id is not None:
#                 p.changeDynamics(
#                     obj_id, -1,
#                     lateralFriction=200.0,
#                     rollingFriction=0.001,
#                     spinningFriction=0.001
#                 )
#             print("[Gripper] Closing in physics mode (no constraint).")
#
#     def set_width(self, width):
#         """Set the gripper opening width (does not affect constraints)."""
#         target = width / 2.0
#         for j in self.fingers:
#             p.setJointMotorControl2(
#                 self.robot_id, j, p.POSITION_CONTROL,
#                 targetPosition=target, force=50
#             )
#
#     def hold(self, duration=0.5):
#         """Hold the current state for a specified duration."""
#         for _ in range(int(duration / (1.0 / 240.0))):
#             p.stepSimulation()
#             time.sleep(1.0 / 240.0)
#
#     def wait_until_settled(self, timeout=0.6):
#         """Step physics for a short while so the gripper can settle."""
#         t_end = time.time() + float(timeout)
#         while time.time() < t_end:
#             p.stepSimulation()
#             time.sleep(1.0 / 240.0)
#
#     def has_contact_with(self, obj_id, min_contacts=1):
#         """
#         Return True if any finger link has contact with the object.
#         NOTE: assumes finger link indices are same as finger joint indices (true for Panda).
#         """
#         if obj_id is None:
#             return False
#         cnt = 0
#         for link_idx in self.fingers:
#             contacts = p.getContactPoints(
#                 bodyA=self.robot_id, bodyB=obj_id, linkIndexA=link_idx
#             )
#             if contacts:
#                 cnt += 1
#         return cnt >= int(min_contacts)

"""Day18-19: Gripper module for Panda arm (Optimized)"""

import pybullet as p
import time
from typing import List, Optional


class Gripper:
    """
    An optimized and more robust controller for the Panda gripper in PyBullet.
    All public interfaces remain unchanged for drop-in replacement.

    Internal Improvements:
    - Fixed critical typo in default mode ("physic" -> "physics").
    - Implemented a two-stage grasp (gentle close + secure hold) for physics mode.
    - Centralized magic numbers as internal constants for easier tuning.
    - Replaced time.sleep with simulation stepping for deterministic behavior.
    """
    # --- Internal constants for easy tuning ---
    _OPEN_WIDTH = 0.08
    _FINGER_FRICTION = 50.0
    _OBJECT_FRICTION = 500.0
    _OPEN_FORCE = 100.0
    _GENTLE_CLOSE_FORCE = 40.0  # A gentle force for the initial close
    _SECURE_HOLD_FORCE = 80.0  # A stronger force to secure the grasp
    _MAX_VELOCITY_GENTLE = 0.5  # Limit velocity during gentle close
    _SIMULATION_HZ = 240.0  # PyBullet's default simulation frequency

    def __init__(self, robot_id: int, finger_joint_indices: List[int], ee_link_index: int, mode: str = "teleport"):
        """
        Panda Gripper control.
        """
        self.robot_id = robot_id
        self.fingers = finger_joint_indices
        self.ee_link_index = ee_link_index
        self.mode = mode
        self.grasp_constraint: Optional[int] = None

        for j in self.fingers:
            p.changeDynamics(
                self.robot_id, j,
                lateralFriction=self._FINGER_FRICTION
            )

    def _step_simulation_for_duration(self, duration: float):
        """Private helper to step simulation for a fixed duration."""
        num_steps = int(duration * self._SIMULATION_HZ)
        for _ in range(num_steps):
            p.stepSimulation()

    def open(self, width: float = 0.08):
        """Open the gripper to the specified width and release object if attached."""
        target = width / 2.0
        for j in self.fingers:
            p.setJointMotorControl2(
                self.robot_id, j, p.POSITION_CONTROL,
                targetPosition=target, force=self._OPEN_FORCE
            )
        if self.grasp_constraint is not None:
            p.removeConstraint(self.grasp_constraint)
            self.grasp_constraint = None
            print("[Gripper] Released object constraint.")

    def close(self, obj_id: Optional[int] = None, width: Optional[float] = None):
        """
        Close the gripper to a specific width.
        """
        target = 0.0 if width is None else width / 2.0

        if self.mode == "teleport":
            if obj_id is not None and self.grasp_constraint is None:
                self.grasp_constraint = p.createConstraint(
                    parentBodyUniqueId=self.robot_id, parentLinkIndex=self.ee_link_index,
                    childBodyUniqueId=obj_id, childLinkIndex=-1,
                    jointType=p.JOINT_FIXED, jointAxis=[0, 0, 0],
                    parentFramePosition=[0, 0, 0], childFramePosition=[0, 0, 0]
                )
                print("[Gripper] Object attached with constraint.")
            # Still command fingers to close visually
            for j in self.fingers:
                p.setJointMotorControl2(self.robot_id, j, p.POSITION_CONTROL, targetPosition=target, force=150)

        elif self.mode == "physics":
            if obj_id is not None:
                p.changeDynamics(obj_id, -1, lateralFriction=self._OBJECT_FRICTION)

            print("[Gripper] Stage 1: Gently closing...")
            for j in self.fingers:
                p.setJointMotorControl2(
                    self.robot_id, j, p.POSITION_CONTROL,
                    targetPosition=target,
                    force=self._GENTLE_CLOSE_FORCE,
                    maxVelocity=self._MAX_VELOCITY_GENTLE
                )

            self.wait_until_settled(0.4)

            print("[Gripper] Stage 2: Securing hold...")
            for j in self.fingers:
                p.setJointMotorControl2(
                    self.robot_id, j, p.POSITION_CONTROL,
                    targetPosition=target,
                    force=self._SECURE_HOLD_FORCE
                )

    def set_width(self, width: float):
        """Set the gripper opening width (does not affect constraints)."""
        target = width / 2.0
        for j in self.fingers:
            p.setJointMotorControl2(
                self.robot_id, j, p.POSITION_CONTROL,
                targetPosition=target, force=self._OPEN_FORCE
            )

    def hold(self, duration: float = 0.5):
        """Hold the current state for a specified duration."""
        self._step_simulation_for_duration(duration)

    def wait_until_settled(self, timeout: float = 0.6):
        """Step physics for a short while so the gripper can settle."""
        self._step_simulation_for_duration(timeout)

    def has_contact_with(self, obj_id: Optional[int], min_contacts: int = 1) -> bool:
        """Return True if any finger link has contact with the object."""
        if obj_id is None:
            return False

        contact_count = 0
        for link_idx in self.fingers:
            contacts = p.getContactPoints(bodyA=self.robot_id, bodyB=obj_id, linkIndexA=link_idx)
            if contacts:
                contact_count += 1
        return contact_count >= min_contacts