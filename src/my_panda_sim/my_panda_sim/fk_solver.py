'''
day8: added Forwardkinematics class. accept target angles to joints, and calculate the transform of ee regarding to base
day11: upgraded FKSolver with bullet_state_guard, support arbitrary link index, and return (pos, quat) or full transform matrix
'''
import pybullet as p
import numpy as np
from contextlib import contextmanager
from scipy.spatial.transform import Rotation as R

@contextmanager
def bullet_state_guard():
    state_id = p.saveState()
    try:
        yield
    finally:
        p.restoreState(state_id)

def to_homogeneous(pos, orn):
    """pos, quat -> 4x4 matrix"""
    Rm = np.array(p.getMatrixFromQuaternion(orn)).reshape(3,3)
    T = np.eye(4)
    T[:3,:3] = Rm
    T[:3, 3] = np.array(pos)
    return T

class FKSolver:
    def __init__(self, body_id, joint_indices, ee_link_index, base_link_index=0):
        self.body_id = body_id
        self.joint_indices = joint_indices
        self.ee_link_index = ee_link_index
        self.base_link_index = base_link_index

    def forward(self, q, target_link_index=None, return_matrix=False):
        if target_link_index is None:
            target_link_index = self.ee_link_index

        with bullet_state_guard():
            for i, jidx in enumerate(self.joint_indices):
                p.resetJointState(self.body_id, jidx, float(q[i]))

            # world -> link
            link_state = p.getLinkState(self.body_id, target_link_index, computeForwardKinematics=True)
            w_pos, w_orn = link_state[4], link_state[5]
            T_w_link = to_homogeneous(w_pos, w_orn)

            # world -> base
            base_state = p.getLinkState(self.body_id, self.base_link_index, computeForwardKinematics=True)
            base_pos, base_orn = base_state[4], base_state[5]
            T_w_base = to_homogeneous(base_pos, base_orn)

            # base -> link
            T_base_link = np.linalg.inv(T_w_base) @ T_w_link

            pos = T_base_link[:3, 3]
            Rm = T_base_link[:3, :3]
            from scipy.spatial.transform import Rotation as Rot
            quat = Rot.from_matrix(Rm).as_quat()  # [x,y,z,w]

            return (pos, quat, T_base_link) if return_matrix else (pos, quat)


    def forward_no_protect(self, q, target_link_index=None, return_matrix=False):
        if target_link_index is None:
            target_link_index = self.ee_link_index

        for i, jidx in enumerate(self.joint_indices):
            p.resetJointState(self.body_id, jidx, float(q[i]))

        # world -> link
        link_state = p.getLinkState(self.body_id, target_link_index, computeForwardKinematics=True)
        w_pos, w_orn = link_state[4], link_state[5]
        T_w_link = to_homogeneous(w_pos, w_orn)

        # world -> base（你用 base_link_index=0，對 Panda 是 link0）
        base_state = p.getLinkState(self.body_id, self.base_link_index, computeForwardKinematics=True)
        base_pos, base_orn = base_state[4], base_state[5]
        T_w_base = to_homogeneous(base_pos, base_orn)

        # base -> link
        T_base_link = np.linalg.inv(T_w_base) @ T_w_link

        pos = T_base_link[:3, 3]
        Rm = T_base_link[:3, :3]
        from scipy.spatial.transform import Rotation as Rot
        quat = Rot.from_matrix(Rm).as_quat()  # [x,y,z,w]

        return (pos, quat, T_base_link) if return_matrix else (pos, quat)
