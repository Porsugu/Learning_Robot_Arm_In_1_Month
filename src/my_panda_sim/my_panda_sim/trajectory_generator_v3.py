import numpy as np

class TrajectoryGeneratorV3:
    def __init__(self, steps: int = 50):
        self.steps = steps

    def interpolate(self, start_pos, goal_pos):
        """
        :param start_pos: [x, y, z]
        :param goal_pos: [x, y, z]
        :return: list of waypoint positions
        """
        start_pos = np.array(start_pos)
        goal_pos = np.array(goal_pos)

        traj = np.linspace(start_pos, goal_pos, self.steps).tolist()
        return traj
