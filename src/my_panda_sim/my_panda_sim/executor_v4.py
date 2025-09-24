"""Day18-20: ExecutorV4 with gripper and collision checking (pause/resume)
+ Place-aware hover/descend over a support surface (e.g., tray)
+ Return-to-home after each grip/place (home aligned to task azimuth)
"""

import pybullet as p
import numpy as np
import matplotlib.pyplot as plt
import time


class ExecutorV4:
    def __init__(self, robot_id, ee_link_index, ik_solver, traj_gen, dof=7,
                 step_size=0.02, sleep_time=0.01, gripper=None, collision_checker=None,
                 home_q=None):
        """
        ExecutorV4
        - Executes regular motion, pick (grip), and place actions.
        - Supports IK preSolve alignment (realign before trajectory execution).
        - Integrates CollisionCheckerV2 with pause/resume mechanism.
        - After grip/place, returns to a home pose aligned to the task azimuth.
        """
        self.robot_id = robot_id
        self.ee_link_index = ee_link_index
        self.ik_solver = ik_solver
        self.traj_gen = traj_gen
        self.dof = dof
        self.step_size = step_size
        self.sleep_time = sleep_time
        self.gripper = gripper
        self.collision_checker = collision_checker

        # Default home pose (joints)
        self.home_q = np.array(home_q, dtype=float) if home_q is not None else np.zeros(dof, dtype=float)

        self._paused = False
        self._resume_flag = False

    def resume(self):
        """Manually resume execution after pause."""
        if self._paused:
            print("[Executor] Resuming trajectory execution...")
            self._resume_flag = True


    # Public API
    def execute(self, q_init, goal_pos, down=True, plot=False, print_diff=True,
                move=None, obj_id=None, hover_offset=0.1,
                place_surface_id=None, place_clearance=0.005, place_hover_offset=None):
        """
        Execute one of three modes:
        - move=None   : Regular motion to goal_pos
        - move="grip" : goal_pos is the object's center → hover → open → descend → close → return-to-home (aligned)
        - move="place": If place_surface_id is provided:
                          hover @ (top_z + hover) → descend to (top_z + clearance) → open → return-to-home (aligned)
                        Else:
                          go to goal_pos → open → return-to-home (aligned)
        """
        # -------------------
        # PICK (grip) mode
        # -------------------
        if move == "grip":
            if obj_id is None:
                raise ValueError("move='grip' requires obj_id")

            # Object bounding box → thickness & width
            aabb_min, aabb_max = p.getAABB(obj_id)
            z_thick = (aabb_max[2] - aabb_min[2]) / 2.0
            x_size = aabb_max[0] - aabb_min[0]
            y_size = aabb_max[1] - aabb_min[1]
            obj_width = max(x_size, y_size)

            hover_z = goal_pos[2] + z_thick + hover_offset
            grasp_z = goal_pos[2] + z_thick

            hover_pos = [goal_pos[0], goal_pos[1], hover_z]
            grasp_pos = [goal_pos[0], goal_pos[1], grasp_z]

            print(f"[Executor] Grip mode: obj_width={obj_width:.3f}, z_thick={z_thick:.3f}")

            # Step 1: move to hover (with preSolve)
            q_traj, _ = self._execute_core(q_init, hover_pos, down, plot, print_diff, presolve=True)

            # Step 2: open gripper sized to object width
            if self.gripper:
                self.gripper.set_width(min(obj_width + 0.02, 0.08))
                self.gripper.open()
                time.sleep(0.3)

            # Step 3: descend to grasp (skip preSolve here)
            q_traj2, _ = self._execute_core(q_traj[-1], grasp_pos, down, plot, print_diff, presolve=False)
            q_traj.extend(q_traj2[1:])  # avoid duplicating seam

            # Step 4: close gripper
            if self.gripper:
                grip_width = min(obj_width + 0.005, 0.08)
                print(f"[Executor] Closing gripper to width {grip_width:.3f}")
                self.gripper.set_width(grip_width)
                self.gripper.close(obj_id=obj_id)
                time.sleep(0.5)

            # Return-to-home (aligned)
            q_home_aligned = self._compute_aligned_home(goal_pos)
            self._move_joints_smoothly(np.array(q_traj[-1]), q_home_aligned)
            q_traj.append(q_home_aligned.tolist())

            return q_traj, None

        # PLACE mode
        elif move == "place":
            q_traj_total = []

            if place_surface_id is not None:
                top_z = self._get_body_top_z(place_surface_id)
                phover = place_hover_offset if place_hover_offset is not None else hover_offset

                hover_pos = [goal_pos[0], goal_pos[1], top_z + phover]
                drop_pos = [goal_pos[0], goal_pos[1], top_z + place_clearance]

                print(f"[Executor] Place over surface {place_surface_id}: "
                      f"top_z={top_z:.3f}, hover={phover:.3f}, clr={place_clearance:.3f}")

                # Step 1: move to hover (with preSolve)
                q_traj1, _ = self._execute_core(q_init, hover_pos, down, plot, print_diff, presolve=True)
                q_traj_total.extend(q_traj1)

                # Step 2: descend to drop (skip preSolve)
                q_traj2, _ = self._execute_core(q_traj1[-1], drop_pos, down, plot, print_diff, presolve=False)
                q_traj_total.extend(q_traj2[1:])

                # Step 3: open gripper
                if self.gripper:
                    print("[Executor] Place → Opening gripper.")
                    self.gripper.open()
                    time.sleep(0.3)
            else:
                q_traj1, (traj_ideal, traj_real) = self._execute_core(
                    q_init, goal_pos, down, plot, print_diff, presolve=True
                )
                q_traj_total.extend(q_traj1)
                if self.gripper:
                    print("[Executor] Place (no surface) → Opening gripper.")
                    self.gripper.open()
                    time.sleep(0.3)

            # Return-to-home (aligned)
            q_home_aligned = self._compute_aligned_home(goal_pos)
            self._move_joints_smoothly(np.array(q_traj_total[-1]), q_home_aligned)
            q_traj_total.append(q_home_aligned.tolist())

            return q_traj_total, None

        # Regular motion
        else:
            return self._execute_core(q_init, goal_pos, down, plot, print_diff, presolve=True)

    # Core trajectory execution
    def _execute_core(self, q_init, goal_pos, down=True, plot=False, print_diff=True, presolve=True):
        ls = p.getLinkState(self.robot_id, self.ee_link_index, computeForwardKinematics=True)
        cur_pos = np.array(ls[4])

        # PreSolve alignment (only if presolve=True)
        if presolve:
            aligned, aligned_q_init = self.ik_solver.preSolve(q_init, goal_pos)
            if not aligned:
                print("[Executor] PreSolve: realigning to safe home pose")
                self._move_joints_smoothly(q_init, np.array(aligned_q_init))
                q_init = np.array(aligned_q_init)

                ls = p.getLinkState(self.robot_id, self.ee_link_index, computeForwardKinematics=True)
                cur_pos = np.array(ls[4])

        # Interpolate Cartesian trajectory
        traj_ideal = self.traj_gen.interpolate(cur_pos, goal_pos)
        traj_real = [cur_pos.tolist()]
        q_traj = [q_init]
        q_cur = np.array(q_init, dtype=float)
        errors = []

        for i, target_pos in enumerate(traj_ideal):
            q_sol, _ = self.ik_solver.solve(q_cur, target_pos, down=down)
            q_sol = np.array(q_sol, dtype=float)

            # Collision pause
            if self.collision_checker and self.collision_checker.detect_collision():
                print("[Executor] Collision detected → Pausing execution.")
                self._paused = True
                self._resume_flag = False
                while not self._resume_flag:
                    p.stepSimulation()
                    time.sleep(1.0 / 240.0)
                print("[Executor] Resumed execution.")
                self._paused = False

            # Execute
            self._move_joints_smoothly(q_cur, q_sol)
            q_cur = q_sol.copy()
            q_traj.append(q_cur.tolist())

            # Record EE pose
            ls = p.getLinkState(self.robot_id, self.ee_link_index, computeForwardKinematics=True)
            real_pos = np.array(ls[4])
            traj_real.append(real_pos.tolist())

            pos_diff = np.linalg.norm(target_pos - real_pos)
            errors.append(pos_diff)
            if print_diff:
                print(f"Step {i}: target={target_pos}, real={real_pos}, diff={pos_diff:.6f}")

        if errors:
            print("\n=== Trajectory Error Summary ===")
            print(f"Mean Error: {np.mean(errors):.6f}")
            print(f"Max Error : {np.max(errors):.6f}")
            print(f"Min Error : {np.min(errors):.6f}")

        if plot:
            self.plot_trajectory(traj_ideal, traj_real)

        return q_traj, (traj_ideal, traj_real)

    # ================================================================
    # Helpers
    # ================================================================
    def _move_joints_smoothly(self, q_start, q_end):
        q_start = np.array(q_start, dtype=float)
        q_end = np.array(q_end, dtype=float)

        diff = q_end - q_start
        n_steps = int(np.max(np.abs(diff)) / self.step_size) + 1
        n_steps = max(n_steps, 2)

        for alpha in np.linspace(0, 1, n_steps):
            q_interp = (1 - alpha) * q_start + alpha * q_end
            for j in range(self.dof):
                p.resetJointState(self.robot_id, j, float(q_interp[j]), targetVelocity=0.0)
            p.stepSimulation()
            time.sleep(self.sleep_time)

    def _get_body_top_z(self, body_id: int) -> float:
        aabb_min, aabb_max = p.getAABB(body_id)
        return aabb_max[2]

    def _compute_aligned_home(self, task_pos_xyz):
        aligned, aligned_q = self.ik_solver.preSolve(self.home_q, np.array(task_pos_xyz, dtype=float))
        if not aligned:
            return np.array(aligned_q, dtype=float)
        return self.home_q.copy()

    def plot_trajectory(self, traj_ideal, traj_real):
        traj_ideal = np.array(traj_ideal)
        traj_real = np.array(traj_real)

        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")
        ax.plot(traj_ideal[:, 0], traj_ideal[:, 1], traj_ideal[:, 2], "b--", label="Ideal")
        ax.scatter(traj_ideal[:, 0], traj_ideal[:, 1], traj_ideal[:, 2], c="b", marker="o")
        ax.plot(traj_real[:, 0], traj_real[:, 1], traj_real[:, 2], "r-", label="Real")
        ax.scatter(traj_real[:, 0], traj_real[:, 1], traj_real[:, 2], c="r", marker="x")

        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.legend()
        ax.set_title("Trajectory Comparison")
        plt.show()
