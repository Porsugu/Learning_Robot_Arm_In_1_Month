"""
Day18: Simple collision tracker for a robot in PyBullet.
"""

import pybullet as p

class CollisionTracker:
    def __init__(self, robot_id: int, obstacle_ids: list, safe_distance: float = 0.0):
        """
        Initialize the tracker.
        :param robot_id: Unique ID of the robot body.
        :param obstacle_ids: List of obstacle body IDs to check against.
        :param safe_distance: Minimum distance threshold (0.0 = only contact).
        """
        self.robot_id = robot_id
        self.obstacle_ids = obstacle_ids
        self.safe_distance = float(safe_distance)

    def check(self) -> bool:
        """
        Check if the robot is colliding (or closer than safe_distance)
        with any obstacle.
        :return: True if collision/too close, False if safe.
        """
        for obs_id in self.obstacle_ids:
            pts = p.getClosestPoints(self.robot_id, obs_id, self.safe_distance)
            if len(pts) > 0:
                return True
        return False
