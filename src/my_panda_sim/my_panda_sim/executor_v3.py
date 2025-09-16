import pybullet as p
import numpy as np
import matplotlib.pyplot as plt
import time

class ExecutorV3:
    def __init__(self, robot_id, ee_link_index, ik_solver, traj_gen, dof=7,
                 step_size=0.02, sleep_time=0.01):
        self.robot_id = robot_id
        self.ee_link_index = ee_link_index
        self.ik_solver = ik_solver
        self.traj_gen = traj_gen
        self.dof = dof
        self.step_size = step_size
        self.sleep_time = sleep_time

    def execute(self, q_init, goal_pos, down=False, plot=True, print_diff=True):
        # Curr ee
        ls = p.getLinkState(self.robot_id, self.ee_link_index, computeForwardKinematics=True)
        cur_pos = np.array(ls[4])

        # insert ideal path
        traj_ideal = self.traj_gen.interpolate(cur_pos, goal_pos)
        traj_real = [cur_pos.tolist()]

        q_traj = [q_init]
        q_cur = np.array(q_init, dtype=float)
        errors = []

        # waypoint
        for i in range(1, len(traj_ideal)):
            target_pos = np.array(traj_ideal[i])

            # IK
            q_sol, _ = self.ik_solver.solve(q_cur, target_pos, down=down)
            q_sol = np.array(q_sol, dtype=float)
            self._move_joints_smoothly(q_cur, q_sol)

            # update joint
            q_cur = q_sol.copy()
            q_traj.append(q_cur.tolist())

            # real ee
            ls = p.getLinkState(self.robot_id, self.ee_link_index, computeForwardKinematics=True)
            real_pos = np.array(ls[4])
            traj_real.append(real_pos.tolist())


            pos_diff = np.linalg.norm(target_pos - real_pos)
            errors.append(pos_diff)
            if print_diff:
                print(f"Step {i}: target={target_pos}, real={real_pos}, diff={pos_diff:.6f}")

        # report
        if errors:
            mean_err = np.mean(errors)
            max_err = np.max(errors)
            min_err = np.min(errors)
            print("\n=== Trajectory Error Summary ===")
            print(f"Mean Error: {mean_err:.6f}")
            print(f"Max Error : {max_err:.6f}")
            print(f"Min Error : {min_err:.6f}")

        if plot:
            self.plot_trajectory(traj_ideal, traj_real)


        self._freeze_kinematic(self.robot_id, self.dof, q_cur, frames=60)

        return q_traj, (traj_ideal, traj_real)


    #Helpers
    def _move_joints_smoothly(self, q_start, q_end):
        """
        :param q_start
        :param q_end
        """
        diff = q_end - q_start
        n_steps = int(np.max(np.abs(diff)) / self.step_size) + 1

        for alpha in np.linspace(0, 1, n_steps):
            q_interp = (1 - alpha) * q_start + alpha * q_end
            for j in range(self.dof):
                p.resetJointState(self.robot_id, j, float(q_interp[j]), targetVelocity=0.0)
            p.stepSimulation()
            time.sleep(self.sleep_time)

    #=======================================================================
    def _freeze_kinematic(self, robot_id, dof, q_target, frames=240):
        p.setJointMotorControlArray(
            bodyUniqueId=robot_id,
            jointIndices=range(dof),
            controlMode=p.POSITION_CONTROL,
            targetPositions=q_target,
            positionGains=[0.01]*dof,  # kp
            velocityGains=[0.1]*dof    # kd
        )

        for _ in range(frames):
            p.stepSimulation()
            time.sleep(1. / 240.)

    def plot_trajectory(self, traj_ideal, traj_real):
        traj_ideal = np.array(traj_ideal)
        traj_real = np.array(traj_real)

        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")
        ax.plot(traj_ideal[:,0], traj_ideal[:,1], traj_ideal[:,2], "b--", label="Ideal")
        ax.scatter(traj_ideal[:,0], traj_ideal[:,1], traj_ideal[:,2], c="b", marker="o")
        ax.plot(traj_real[:,0], traj_real[:,1], traj_real[:,2], "r-", label="Real")
        ax.scatter(traj_real[:,0], traj_real[:,1], traj_real[:,2], c="r", marker="x")

        ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")
        ax.legend(); ax.set_title("Trajectory Comparison")
        plt.show()

