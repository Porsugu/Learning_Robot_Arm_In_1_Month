'''
day9: matrix chain, for computing the combined transformation
day10: handle edge cases, pack the whole class as a separate helper module
'''
import numpy as np

class MatrixChain:
    """
        A helper class to manage and compute a chain of homogeneous transforms.
        """
    def __init__(self, base_frame):
        self.base_frame = base_frame
        self.trans = []

    def addTrans(self, joint_name, T):
        if T.shape != (4, 4):
            raise ValueError("Transform must be a 4x4 matrix")
        self.trans.append((joint_name, T))

    def compute_chain_to(self, joint_index):
        if joint_index >= len(self.trans):
            raise IndexError("Joint index out of range")

        total_Trans = np.eye(4)
        for name, T in self.trans:
            total_Trans = total_Trans @ T

        return total_Trans

    def compute_end(self):
        """
        Compute transform from base to the last joint (end-effector).
        """
        return self.compute_chain_to(len(self.trans) - 1)

    def reset(self):
        """Clear all transforms."""
        self.transforms = []


    def computeChainDebugger(self):
        total_Trans = np.eye(4)
        print('Chain: ',self.trans)
        print('Init matrix: ', total_Trans)

        for name, T in self.trans:
            print('--------------')
            print("Processing: ", name)
            print("Before chain: ", total_Trans)
            total_Trans = total_Trans @ T
            print('After chain: ', total_Trans)
        print('--------------')
        return total_Trans
