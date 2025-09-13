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
    def __init__(self, body_id, joint_indices, base_link_index=0):
        self.body_id = body_id
        self.joint_indices = joint_indices
        self.base_link_index = base_link_index

    def forward(self, q, target_link_index, return_matrix=False):
        """
        input:
            q (list/ndarray): joint angles
            target_link_index (int): link index (e.g. 11 for panda_hand)
            return_matrix (bool): if True, also return 4x4 transform matrix

        output:
            pos (np.array, shape (3,))
            quat (np.array, shape (4,), [x,y,z,w])
            (optional) T_base_link (np.array, shape (4,4))
        """
        with bullet_state_guard():
            p.setJointMotorControlArray(
                self.body_id,
                self.joint_indices,
                p.POSITION_CONTROL,
                targetPositions=q
            )

            #Get link pos and orn that is base on the world
            # world -> link
            link_state = p.getLinkState(self.body_id, target_link_index, computeForwardKinematics=True)
            w_pos, w_orn = link_state[4], link_state[5]
            T_w_link = to_homogeneous(w_pos, w_orn)

            # Get T_w_base
            # world -> base
            base_state = p.getLinkState(self.body_id, self.base_link_index, computeForwardKinematics=True)
            base_pos, base_orn = base_state[4], base_state[5]
            T_w_base = to_homogeneous(base_pos, base_orn)

            #T_w_base @ T_base_link = T_world_base
            # base -> link
            T_base_link = np.linalg.inv(T_w_base) @ T_w_link

            pos = T_base_link[:3, 3]
            Rm = T_base_link[:3, :3]
            quat = R.from_matrix(Rm).as_quat()   # [x, y, z, w]

            if return_matrix:
                return pos, quat, T_base_link
            else:
                return pos, quat
