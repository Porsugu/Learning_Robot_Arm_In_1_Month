import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation as R

import utils
from src.my_panda_sim.my_panda_sim.utils import to_homogeneous


class ForwardKinematics:
    def __init__(self, robot_id, joint_indexes,ee_link_index,  T_w_b = None):
        self.robot_id = robot_id
        self.joint_indexes = joint_indexes
        self.ee_link_index = ee_link_index if ee_link_index is not None else np.eye(4)
        self.T_w_b = T_w_b

    def forward(self, q):
        for index, angle in zip(self.joint_indexes, q):
            p.resetJointState(self.robot_id,index,angle)

        ls = p.getLinkState(self.robot_id, self.ee_link_index)
        pos_w, orn_w = ls[4], ls[5]

        T_w_ee = to_homogeneous(pos_w, orn_w)
        T_b_ee = np.linalg.inv(self.T_w_b) @ T_w_ee

        pos_b_ee = T_b_ee[:3, 3]
        orn_b_ee = R.from_matrix(T_b_ee[:3, :3]).as_quat()

        return np.array(pos_b_ee), np.array(orn_b_ee)