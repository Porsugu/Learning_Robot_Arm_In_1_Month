"""Day18-19: Gripper module for Panda arm"""

import pybullet as p
import time


class Gripper:
    def __init__(self, robot_id, finger_joint_indices, ee_link_index):
        """
        Panda Gripper control (constraint-based gripping).

        Args:
            robot_id (int): Panda robot unique id in PyBullet
            finger_joint_indices (list[int]): Joint indices for the fingers (usually [9, 10])
            ee_link_index (int): End-effector link index (usually 11)
        """
        self.robot_id = robot_id
        self.fingers = finger_joint_indices
        self.ee_link_index = ee_link_index
        self.grasp_constraint = None

    def open(self, width=0.08):
        """
        Open the gripper to the specified width.
        Releases any attached object if a constraint exists.
        """
        target = width / 2
        for j in self.fingers:
            p.setJointMotorControl2(
                self.robot_id, j, p.POSITION_CONTROL,
                targetPosition=target, force=50
            )
        if self.grasp_constraint is not None:
            p.removeConstraint(self.grasp_constraint)
            self.grasp_constraint = None
            print("[Gripper] Released object.")

    def close(self, obj_id=None):
        """
        Close the gripper.
        If obj_id is provided, creates a fixed constraint to attach the object.

        Args:
            obj_id (int, optional): PyBullet body id of the object to grasp
        """
        for j in self.fingers:
            p.setJointMotorControl2(
                self.robot_id, j, p.POSITION_CONTROL,
                targetPosition=0.0, force=50
            )

        if obj_id is not None and self.grasp_constraint is None:
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

    def set_width(self, width):
        """
        Set the gripper opening width.
        This only changes finger joint positions and does not affect constraints.
        """
        target = width / 2
        for j in self.fingers:
            p.setJointMotorControl2(
                self.robot_id, j, p.POSITION_CONTROL,
                targetPosition=target, force=50
            )

    def hold(self, duration=0.5):
        """
        Hold the current state for a specified duration.

        Args:
            duration (float): Time in seconds to hold
        """
        for _ in range(int(duration / (1.0 / 240.0))):
            p.stepSimulation()
            time.sleep(1.0 / 240.0)
