"""Day17: executor"""
import pybullet as p
import time


class Executor:
    def __init__(self, robot_id, joint_indices, dt=0.01):
        self.robot_id = robot_id
        self.joint_indices = joint_indices
        self.dt = dt

    def execute_trajectory(self, trajectory):
        """
        trajectory: numpy array [steps, dof]
        """
        for q in trajectory:
            for i, joint_index in enumerate(self.joint_indices):
                p.setJointMotorControl2(
                    bodyIndex=self.robot_id,
                    jointIndex=joint_index,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=float(q[i]),
                    force=200
                )
            p.stepSimulation()
            time.sleep(self.dt)
