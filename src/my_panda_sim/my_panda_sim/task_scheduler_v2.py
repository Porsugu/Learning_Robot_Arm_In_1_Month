"""Day21: TaskScheduler v2 with dynamic task queue"""

import pybullet as p
import time
from collections import deque


class TaskSchedulerV2:
    def __init__(self, executor, ik_solver, drop_pos=None):
        """
        TaskScheduler v2: supports dynamic task queue.
        :param executor: ExecutorV4 instance
        :param ik_solver: IK solver instance
        :param drop_pos: default drop-off location [x, y, z]
        """
        self.executor = executor
        self.ik_solver = ik_solver
        self.drop_pos = drop_pos
        self.task_queue = deque()

    def add_task(self, obj_id, pick_pos=None, place_pos=None):
        """
        Add a new pick-and-place task.
        :param obj_id: object ID
        :param pick_pos: [x, y, z] (if None, use object base position)
        :param place_pos: [x, y, z] (if None, use self.drop_pos)
        """
        if pick_pos is None:
            pick_pos, _ = p.getBasePositionAndOrientation(obj_id)
        if place_pos is None:
            place_pos = self.drop_pos
        self.task_queue.append({
            "obj_id": obj_id,
            "pick_pos": pick_pos,
            "place_pos": place_pos
        })
        print(f"[Scheduler] Added task: pick at {pick_pos}, place at {place_pos}")

    def task_count(self):
        return len(self.task_queue)

    def has_next(self):
        return len(self.task_queue) > 0

    def run_next(self, q_cur):
        """
        Execute the next task in the queue.
        :param q_cur: current joint state
        :return: final joint state
        """
        if not self.has_next():
            print("[Scheduler] No tasks in queue.")
            return q_cur

        task = self.task_queue.popleft()
        obj_id = task["obj_id"]
        pick_pos = task["pick_pos"]
        place_pos = task["place_pos"]

        print(f"[Scheduler] Running task: pick {obj_id} at {pick_pos} → place at {place_pos}")

        # Pick
        q_traj, _ = self.executor.execute(q_cur, pick_pos, move="grip", obj_id=obj_id, plot=False, print_diff=False)
        q_cur = q_traj[-1]

        # Place
        q_traj, _ = self.executor.execute(q_cur, place_pos, move="place", plot=False, print_diff=False)
        q_cur = q_traj[-1]

        print("[Scheduler] Task completed.")
        return q_cur

    def run_all(self, q_cur):
        """
        Run until all tasks in the queue are done.
        """
        while self.has_next():
            q_cur = self.run_next(q_cur)
        print("[Scheduler] All tasks completed.")
        return q_cur
