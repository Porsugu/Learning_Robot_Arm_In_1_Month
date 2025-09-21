"""Day20: CollisionCheckerV2"""

import pybullet as p


class CollisionCheckerV2:
    def __init__(self, robot_id, obstacle_ids=None):
        """
        Simple collision checker for trajectories.

        Args:
            robot_id (int): PyBullet body id of the robot
            obstacle_ids (list[int], optional): List of obstacle ids to check against
        """
        self.robot_id = robot_id
        self.obstacle_ids = obstacle_ids if obstacle_ids else []

    def detect_collision(self):
        """
        Check if the robot is currently colliding with any obstacle or itself.

        Returns:
            bool: True if collision detected, False otherwise
        """
        # Check collisions with external obstacles
        for obs_id in self.obstacle_ids:
            contacts = p.getContactPoints(self.robot_id, obs_id)
            if len(contacts) > 0:
                return True

        # Self-collision check
        self_contacts = p.getContactPoints(self.robot_id, self.robot_id)
        if len(self_contacts) > 0:
            return True

        return False

    def check_trajectory(self, joint_traj, joint_indices):
        """
        Check if a full joint trajectory is collision-free (offline check).

        Args:
            joint_traj (list[list[float]]): Sequence of joint positions
            joint_indices (list[int]): Joint indices to reset for each waypoint

        Returns:
            bool: True if safe, False if collision detected
        """
        for q in joint_traj:
            for j, val in zip(joint_indices, q):
                p.resetJointState(self.robot_id, j, val)

            if self.detect_collision():
                return False

        return True
