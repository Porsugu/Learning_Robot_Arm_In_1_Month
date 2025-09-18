import pybullet as p
import numpy as np


class CollisionCheckerV2:
    def __init__(self, robot_id, ee_link_index, obstacles=None, dof=7, safe_dist=0.01):
        """
        robot_id: Panda 機械臂 ID
        ee_link_index: 末端執行器的 link index
        obstacles: 障礙物的 body_id list
        dof: 關節數 (預設 7)
        safe_dist: 最近點小於這個距離就視為危險
        """
        self.robot_id = robot_id
        self.ee_link_index = ee_link_index
        self.obstacles = obstacles if obstacles is not None else []
        self.dof = dof
        self.safe_dist = safe_dist

    def add_obstacle(self, body_id):
        """加入新的障礙物"""
        self.obstacles.append(body_id)

    def check_state(self, q):
        """
        給定一組關節角度 q，檢查是否發生碰撞
        return: (bool, details)
        """
        assert len(q) == self.dof

        # 設定機械臂狀態
        for i in range(self.dof):
            p.resetJointState(self.robot_id, i, q[i])

        # 檢查與所有障礙物的最短距離
        for obs_id in self.obstacles:
            pts = p.getClosestPoints(self.robot_id, obs_id, self.safe_dist)
            if len(pts) > 0:  # 有接觸或進入安全距離
                return True, pts
        return False, None

    def check_trajectory(self, traj):
        """
        給定整個 trajectory (list of q)，逐點檢查
        return: 碰撞點的 index list
        """
        collision_points = []
        for i, q in enumerate(traj):
            collision, details = self.check_state(q)
            if collision:
                collision_points.append(i)
        return collision_points
