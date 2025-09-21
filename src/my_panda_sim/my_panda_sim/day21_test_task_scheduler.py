"""Demo: Spawn cubes with PyQt5 button + task queue (home→pick→place→home per task)"""

import sys
import random
import time
import threading
import numpy as np

import pybullet as p
import pybullet_data
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QLabel

from trajectory_generator_v3 import TrajectoryGeneratorV3
from ik_solver_v2 import IKSolverV2
from gripper import Gripper
from executor_v4 import ExecutorV4
from task_scheduler_v2 import TaskSchedulerV2


class CubeDemo:
    def __init__(self):
        # Init PyBullet
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)

        p.loadURDF("plane.urdf")
        self.robot_id = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

        ee_link_index = 11
        dof = 7
        finger_joints = [9, 10]

        # Core modules
        self.ik_solver = IKSolverV2(self.robot_id, ee_link_index, dof)
        self.traj_gen = TrajectoryGeneratorV3(steps=10)
        self.gripper = Gripper(self.robot_id, finger_joints, ee_link_index)
        self.executor = ExecutorV4(self.robot_id, ee_link_index, self.ik_solver,
                                   self.traj_gen, dof,
                                   gripper=self.gripper, collision_checker=None)

        # Home pose
        q_home = np.array(self.ik_solver.home_pose)
        for j in range(dof):
            p.resetJointState(self.robot_id, j, q_home[j])
        print("[Demo] Robot at home pose.")

        # Drop-off tray
        drop_box = p.loadURDF("tray/traybox.urdf", [0.0, 0.5, 0.0], useFixedBase=True)
        p.changeVisualShape(drop_box, -1, rgbaColor=[0.8, 0.8, 0.8, 1])
        self.drop_pos = [0.0, 0.5, 0.15]

        # State variables
        self.q_cur = q_home
        self.total_generated = 0
        self.total_completed = 0

        # Queue of tasks
        self.task_queue = []
        self.spawned_cubes = []
        self.min_dist = 0.08  # Minimum cube spacing

        # GUI label reference
        self.gui_label = None

    def _sample_position(self):
        """Generate a random valid cube position"""
        while True:
            x = random.uniform(0.3, 0.7)
            # Allow both front and back regions
            if random.random() < 0.5:
                y = random.uniform(-0.5, -0.2)  # Back
            else:
                y = random.uniform(0.0, 0.3)    # Front
            pos = [x, y, 0.02]

            # Avoid the tray area
            if abs(x - self.drop_pos[0]) < 0.2 and abs(y - self.drop_pos[1]) < 0.2:
                continue

            # Avoid too-close cubes
            too_close = False
            for cid in self.spawned_cubes:
                cur_pos = p.getBasePositionAndOrientation(cid)[0]
                dist = np.linalg.norm(np.array(cur_pos[:2]) - np.array(pos[:2]))
                if dist < self.min_dist:
                    too_close = True
                    break
            if too_close:
                continue

            return pos

    def spawn_cube(self):
        """Spawn a cube and push a task into the queue"""
        pos = self._sample_position()
        obj_id = p.loadURDF("cube_small.urdf", pos)
        p.changeVisualShape(obj_id, -1, rgbaColor=[0, 1, 0, 1])

        self.task_queue.append({"id": obj_id, "pick_pos": pos})
        self.spawned_cubes.append(obj_id)
        self.total_generated += 1
        print(f"[Demo] Added task for cube at {pos}")

        # Update GUI
        if self.gui_label is not None:
            status_text = (f"Task Status | "
                           f"Generated: {self.total_generated}, "
                           f"Completed: {self.total_completed}, "
                           f"Pending: {len(self.task_queue)}")
            self.gui_label.setText(status_text)

    def run(self):
        """Background simulation loop"""
        while True:
            if self.task_queue:
                # Get a task
                task = self.task_queue.pop(0)
                obj_id = task["id"]
                pick_pos = task["pick_pos"]

                print(f"[Demo] Executing task for cube {obj_id} at {pick_pos}")

                # Step 1: pick
                q_traj, _ = self.executor.execute(
                    self.q_cur, pick_pos, move="grip", obj_id=obj_id,
                    down=True, plot=False, print_diff=False
                )
                self.q_cur = q_traj[-1]

                # Step 2: place (into tray)
                q_traj, _ = self.executor.execute(
                    self.q_cur, self.drop_pos, move="place",
                    place_surface_id=None,  # If you want tray surface alignment, pass drop_box id
                    down=True, plot=False, print_diff=False
                )
                self.q_cur = q_traj[-1]

                self.total_completed += 1
                print(f"[Demo] Task for cube {obj_id} completed.")

            else:
                # Idle when no task
                p.stepSimulation()
                time.sleep(1.0 / 240.0)

            # Update status
            status_text = (f"Task Status | "
                           f"Generated: {self.total_generated}, "
                           f"Completed: {self.total_completed}, "
                           f"Pending: {len(self.task_queue)}")
            print(status_text, end="\r")

            if self.gui_label is not None:
                self.gui_label.setText(status_text)


class ControlWindow(QWidget):
    def __init__(self, demo: CubeDemo):
        super().__init__()
        self.demo = demo
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle("PyBullet Control Panel")
        self.setGeometry(200, 200, 300, 150)

        layout = QVBoxLayout()

        spawn_btn = QPushButton("Spawn Cube")
        spawn_btn.clicked.connect(self.demo.spawn_cube)
        layout.addWidget(spawn_btn)

        # Status label
        self.status_label = QLabel("Task Status | Generated: 0, Completed: 0, Pending: 0")
        layout.addWidget(self.status_label)

        self.setLayout(layout)
        self.demo.gui_label = self.status_label


def main():
    demo = CubeDemo()

    sim_thread = threading.Thread(target=demo.run, daemon=True)
    sim_thread.start()

    app = QApplication(sys.argv)
    control = ControlWindow(demo)
    control.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
