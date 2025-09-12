'''
Day8: added to homogeneous
'''
import pybullet as p
import numpy as np

def to_homogeneous(pos, orn):
    R = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.array(pos)
    return T