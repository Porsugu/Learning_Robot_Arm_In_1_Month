import pybullet as p
import pybullet_data
import random
import numpy as np
import time


class CubeDemo:
    def __init__(self):
        # 初始化 PyBullet
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)

        # 加地面 & Panda
        p.loadURDF("plane.urdf")
        self.robot_id = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

        # 投放點（tray）
        self.drop_pos = [0.0, 0.5, 0.15]
        drop_box = p.loadURDF("tray/traybox.urdf", [0.0, 0.5, 0.0], useFixedBase=True)
        p.changeVisualShape(drop_box, -1, rgbaColor=[0.8, 0.8, 0.8, 1])

        # 狀態
        self.task_queue = []         # [{id, pick_pos}, ...]
        self.spawned_cubes = []      # 已生成的 cube ID
        self.min_dist = 0.08         # cube 間最小距離
        self.total_generated = 0
        self.total_completed = 0

    # ------------------------------------------------
    # 隨機生成 cube 的位置
    # ------------------------------------------------
    def _sample_position(self):
        while True:
            x = random.uniform(0.3, 0.7)

            # 允許在前面或後面
            if random.random() < 0.5:
                y = random.uniform(-0.5, -0.2)  # 後區域
            else:
                y = random.uniform(0.0, 0.3)    # 前區域

            pos = [x, y, 0.02]

            # 避開 tray 區域
            if abs(x - self.drop_pos[0]) < 0.2 and abs(y - self.drop_pos[1]) < 0.2:
                continue

            # 避開太近的 cube
            too_close = False
            for cid in self.spawned_cubes:
                cur_pos = p.getBasePositionAndOrientation(cid)[0]
                if np.linalg.norm(np.array(cur_pos[:2]) - np.array(pos[:2])) < self.min_dist:
                    too_close = True
                    break

            if not too_close:
                return pos

    # ------------------------------------------------
    # make cube
    # ------------------------------------------------
    def spawn_cube(self):
        pos = self._sample_position()
        obj_id = p.loadURDF("cube_small.urdf", pos)
        p.changeVisualShape(obj_id, -1, rgbaColor=[0, 1, 0, 1])

        self.task_queue.append({"id": obj_id, "pick_pos": pos})
        self.spawned_cubes.append(obj_id)
        self.total_generated += 1

        print(f"[Demo] Added task for cube at {pos}")
        return obj_id

    # ------------------------------------------------
    # Main loop
    # ------------------------------------------------
    def run(self):
        while True:
            if self.task_queue:
                # Get mission
                task = self.task_queue.pop(0)
                obj_id = task["id"]
                pick_pos = task["pick_pos"]

                print(f"[Demo] Executing task for cube {obj_id} at {pick_pos}")

                # TODO: Executor → pick/place
                time.sleep(1.0)
                self.total_completed += 1
                print(f"[Demo] Task for cube {obj_id} completed.")

            else:
                # No Mission, keep simulate
                p.stepSimulation()
                time.sleep(1.0 / 240.0)

            status_text = (f"Task Status | "
                           f"Generated: {self.total_generated}, "
                           f"Completed: {self.total_completed}, "
                           f"Pending: {len(self.task_queue)}")
            print(status_text, end="\r")
