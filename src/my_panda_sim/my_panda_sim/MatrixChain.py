'''
day9: matrix chain, for computing the combined transformation
'''
import numpy as np

class MatrixChain:
    def __init__(self):
        self.trans = []

    def addTrans(self, name, T):
        self.trans.append((name, T))

    def computeChain(self):
        total_Trans = np.eye(4)
        for name, T in self.trans:
            total_Trans = total_Trans @ T

        return total_Trans

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
