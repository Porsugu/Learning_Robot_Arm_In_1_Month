#0 = JOINT_REVOLUTE

# 1 = JOINT_PRISMATIC
#
# 2 = JOINT_SPHERICAL
#
# 3 = JOINT_PLANAR
#
# 4 = JOINT_FIXED

import pybullet as p
import pybullet_data

p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

robot_id = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

num_joints = p.getNumJoints(robot_id)
for j in range(num_joints):
    info = p.getJointInfo(robot_id, j)
    joint_index = info[0]
    joint_name = info[1].decode("utf-8")
    joint_type = info[2]
    print(f"Joint {joint_index}: {joint_name}, type={joint_type}")
