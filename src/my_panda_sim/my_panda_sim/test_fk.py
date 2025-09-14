"""
Day 8: FK tester
"""

import pybullet as p
import pybullet_data
from fk_solver import FKSolver

# ---------------- PyBullet----------------
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# loadPanda URDF
robot_id = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

#arm joints
joint_indices = list(range(7))
# Panda hand link index
ee_link_index = 11

# init FK solver
fk = FKSolver(robot_id, joint_indices, ee_link_index)

# testing
# q_test = [0, 0, 0, 0, 0, 1, 0]
# q_test = [0.0, 0.5, 0.0, -1.2, 0.0, 1.0, 1.5]
# q_test = [-4.88865018e-01, -1.65808778e-01, -5.15124992e-01, -2.98605834e+00, -2.89341586e-03, 7.99101552e-01, 2.80385930e+00]
# q_test = [ 0.27543053, -1.173592, 0.30232506, -1.96348611, -2.82903249,  3.40890657, 0.0256137 ]
# q_test =  [-8.94688115e-12,  1.15545699e+00, -8.48002233e-13, -9.78888777e-02, -2.77662376e-12,  3.33604431e-01,  0.00000000e+00]
# q_test =  [-8.94692628e-12,  1.15545699e+00, -8.48000588e-13, -9.78888777e-02, -2.77661876e-12,  3.33604431e-01,  1.37729882e+00]
q_test = [-8.94691401e-12,  1.15545699e+00, -8.48021703e-13, -9.78888777e-02, -2.77663860e-12,  3.33604431e-01,  1.41101562e+00]
q_test = [-5.26787936e-12,  1.15975548e+00,  4.67125488e-12, -8.78370270e-02, 3.04183743e-12,  3.31524210e-01,  0.00000000e+00]
# FK
pos, quat = fk.forward_no_protect(q_test)

print("End-effector position:", pos)
print("End-effector quaternion [x,y,z,w]:", quat)

input("Press Enter to exit...")
p.disconnect()
