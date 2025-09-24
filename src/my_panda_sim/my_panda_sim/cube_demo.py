import time
import random
import numpy as np

import pybullet as p
import pybullet_data

from .trajectory_generator_v3 import TrajectoryGeneratorV3
from .ik_solver_v2 import IKSolverV2
from .gripper import Gripper
from .executor_v4 import ExecutorV4

from my_panda_interfaces.msg import TaskStatus


class CubeDemo:
    """
    ROS-based task queue demo:
      - spawn_cube(): create a cube and add a pick task
      - run(): background thread executes tasks (home → pick → place → home)
      - set_autorun / get_status / set_drop_pos
      - publishes TaskStatus to /task_status whenever tasks are updated
    """

    def __init__(self, node):
        # ROS2 node and publisher
        self.node = node
        self.status_pub = self.node.create_publisher(TaskStatus, 'task_status', 10)

        # PyBullet setup
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)

        p.loadURDF("plane.urdf")
        self.robot_id = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

        # Panda robot setup
        ee_link_index = 11
        dof = 7
        finger_joints = [9, 10]

        self.ik_solver = IKSolverV2(self.robot_id, ee_link_index, dof)
        self.traj_gen = TrajectoryGeneratorV3(steps=10)
        self.gripper = Gripper(self.robot_id, finger_joints, ee_link_index)

        self.executor = ExecutorV4(
            self.robot_id,
            ee_link_index,
            self.ik_solver,
            self.traj_gen,
            dof,
            gripper=self.gripper,
            collision_checker=None
        )

        # Home pose
        self.q_home = np.array(self.ik_solver.home_pose)
        for j in range(dof):
            p.resetJointState(self.robot_id, j, self.q_home[j])
        self.q_cur = self.q_home.copy()
        print("[CubeDemo] Robot moved to home pose.")

        # Tray setup
        # This is now the default or center position for the tray
        self.drop_pos = [0.0, 0.5, 0.15]
        self.drop_box_id = p.loadURDF("tray/traybox.urdf", self.drop_pos, useFixedBase=True)
        p.changeVisualShape(self.drop_box_id, -1, rgbaColor=[0.8, 0.8, 0.8, 1.0])

        # Grid arrangement parameters
        self.grid_cols = 3
        self.grid_rows = 3
        self.grid_spacing_factor = 1.2

        # Task and stats
        self.task_queue = []
        self.spawned_cubes = []
        self.min_dist = 0.08
        self.total_generated = 0
        self.total_completed = 0

        # Autorun toggle
        self.autorun = True

    def _publish_status(self):
        """Publish current task status."""
        msg = TaskStatus()
        msg.total_generated = int(self.total_generated)
        msg.total_completed = int(self.total_completed)
        msg.pending = int(len(self.task_queue))
        self.status_pub.publish(msg)

    def _sample_position(self):
        """Randomly sample a valid cube position."""
        while True:
            x = random.uniform(-0.4, 0.4)
            y = random.uniform(-0.4, -0.25)
            pos = [x, y, 0.02]

            # Use tray center position to avoid spawning there
            if abs(x - self.drop_pos[0]) < 0.2 and abs(y - self.drop_pos[1]) < 0.2:
                continue

            if -0.25 < x < 0.25:
                continue

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

    def spawn_cube(self) -> int:
        """Spawn a cube and add it to the task queue."""
        pos = self._sample_position()
        obj_id = p.loadURDF("cube_small.urdf", pos)
        p.changeVisualShape(obj_id, -1, rgbaColor=[0.0, 1.0, 0.0, 1.0])

        p.changeDynamics(
            obj_id, -1, mass=0.02, lateralFriction=1.5,
            rollingFriction=0.001, spinningFriction=0.001, restitution=0.0
        )

        self.task_queue.append({"id": obj_id, "pick_pos": pos})
        self.spawned_cubes.append(obj_id)
        self.total_generated += 1

        print(f"[CubeDemo] Added task: cube {obj_id} at {pos}")
        self._publish_status()
        return int(obj_id)

    def set_autorun(self, enable: bool) -> bool:
        """Enable or disable autorun mode."""
        self.autorun = bool(enable)
        print(f"[CubeDemo] autorun set to {self.autorun}")
        self._publish_status()
        return True

    def get_status(self):
        """Return current task statistics."""
        return int(self.total_generated), int(self.total_completed), int(len(self.task_queue))

    def set_drop_pos(self, x: float, y: float, z: float) -> bool:
        """Update the tray's center position."""
        self.drop_pos = [float(x), float(y), float(z)]
        p.resetBasePositionAndOrientation(self.drop_box_id, self.drop_pos, [0, 0, 0, 1])
        print(f"[CubeDemo] drop_pos (tray center) updated to {self.drop_pos}")
        return True

    def run(self):
        """Main loop to process tasks automatically."""
        while True:
            if self.autorun and self.task_queue:
                task = self.task_queue.pop(0)
                obj_id = task["id"]
                pick_pos = task["pick_pos"]

                print(f"\n[CubeDemo] Start task: cube {obj_id} at {pick_pos}")

                try:
                    # Pick
                    q_traj, _ = self.executor.execute(
                        self.q_cur, pick_pos, move="grip", obj_id=obj_id,
                        down=True, plot=False, print_diff=False
                    )
                    self.q_cur = q_traj[-1]

                    # ================================================================= #
                    # 【MODIFIED LOGIC】Dynamically calculate the placement position    #
                    # ================================================================= #
                    n = self.total_completed

                    # 1. Get the dimensions (thickness) of the object being held
                    aabb = p.getAABB(obj_id)
                    cube_dims = np.array(aabb[1]) - np.array(aabb[0])

                    if n >= self.grid_cols * self.grid_rows:
                        print("[CubeDemo] Tray is full! Placing at center.")
                        target_x = self.drop_pos[0]
                        target_y = self.drop_pos[1]
                    else:
                        # 2. Calculate row and column for the grid
                        row = n // self.grid_cols
                        col = n % self.grid_cols

                        spacing = cube_dims[0] * self.grid_spacing_factor

                        # Calculate the top-left starting point of the grid
                        start_x = self.drop_pos[0] - (self.grid_cols - 1) * spacing / 2.0
                        start_y = self.drop_pos[1] - (self.grid_rows - 1) * spacing / 2.0

                        # Calculate the target (x, y) for the current cube
                        target_x = start_x + col * spacing
                        target_y = start_y + row * spacing

                    # 3. Calculate the dynamic Z-axis position
                    # Z = tray_top_surface + object_thickness + clearance
                    tray_aabb = p.getAABB(self.drop_box_id)
                    tray_top_z = tray_aabb[1][2]

                    target_z = tray_top_z + (cube_dims[2] / 2.0) + 0.05

                    # 4. Assemble the final target position vector
                    target_drop_pos = [target_x, target_y, target_z]

                    print(f"[CubeDemo] Placing cube at dynamic grid position: {np.round(target_drop_pos, 3)}")

                    # Place
                    q_traj, _ = self.executor.execute(
                        self.q_cur,
                        target_drop_pos,  # Use the dynamically calculated position
                        move="place",
                        place_surface_id=self.drop_box_id,
                        down=True,
                        plot=False,
                        print_diff=False
                    )
                    self.q_cur = q_traj[-1]

                    self.total_completed += 1
                    print(f"[CubeDemo] Done task: cube {obj_id}. Completed={self.total_completed}")
                    self._publish_status()

                except Exception as e:
                    print(f"[CubeDemo][ERROR] Task for cube {obj_id} failed: {e}")

            else:
                p.stepSimulation()
                time.sleep(1.0 / 240.0)

            status_text = (f"Task Status | "
                           f"Generated: {self.total_generated}, "
                           f"Completed: {self.total_completed}, "
                           f"Pending: {len(self.task_queue)}")
            print(status_text, end="\r")