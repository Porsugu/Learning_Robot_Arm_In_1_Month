"""Day17: TrajectoryGeneratorV3
Generates Cartesian waypoint trajectories.
- Provides linear interpolation between start and goal positions.
- Designed for integration with IK solver and executor.
- Default resolution is controlled by the number of steps.
"""

import numpy as np


class TrajectoryGeneratorV3:
    def __init__(self, steps: int = 50):
        """
        Initialize the trajectory generator.

        Args:
            steps (int): Number of interpolation steps between start and goal.
        """
        self.steps = steps

    def interpolate(self, start_pos, goal_pos):
        """
        Generate a Cartesian trajectory from start_pos to goal_pos.

        Args:
            start_pos (array-like): [x, y, z] starting position in world coordinates.
            goal_pos (array-like): [x, y, z] goal position in world coordinates.

        Returns:
            list[list[float]]: List of waypoints (each [x, y, z]) along the path.
        """
        start_pos = np.array(start_pos, dtype=float)
        goal_pos = np.array(goal_pos, dtype=float)

        traj = np.linspace(start_pos, goal_pos, self.steps).tolist()
        return traj
