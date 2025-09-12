'''
Day 8: tester
'''
import pybullet as p
import pybullet_data
from fk_solver import ForwardKinematics

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

robot_id = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

joint_indexes = [i for i in range(7)]
ee_link_index = 11

fk = ForwardKinematics(robot_id, joint_indexes, ee_link_index)

q_test = [0.0, -0.5, 0.5, -1.0, 0.0, 1.0, 0.5]
pos, quat = fk.forward(q_test)

print("End-effector pos:", pos)
print("End-effector quat:", quat)

input("Press Enter to exit...")
p.disconnect()