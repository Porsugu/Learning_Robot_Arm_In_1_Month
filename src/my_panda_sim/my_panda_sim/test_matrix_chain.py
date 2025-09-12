'''
day9: tester of chain matrix for fk
'''
import numpy as np
from MatrixChain import MatrixChain

def rot_z(theta):
    """A homogeneous matrix of rotate theta rad """
    c, s = np.cos(theta), np.sin(theta)
    T = np.eye(4)
    T[:3,:3] = [[c, -s, 0],
                [s,  c, 0],
                [0,  0, 1]]
    return T

def trans_x(d):
    """A homogeneous matrix for moving along x axis """
    T = np.eye(4)
    T[0,3] = d
    return T

if __name__ == "__main__":
    dbg = MatrixChain()
    dbg.addTrans("Joint1: rot_z(90°)", rot_z(np.pi/2))
    dbg.addTrans("Link1: trans_x(0.3)", trans_x(0.3))
    dbg.addTrans("Joint2: rot_z(45°)", rot_z(np.pi/4))

    dbg.computeChainDebugger()
