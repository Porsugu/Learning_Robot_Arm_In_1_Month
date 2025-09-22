import pybullet as p
import pybullet_data
import numpy as np

def quat_to_matrix(quat):
    """Convert quaternion to rotation matrix (3x3)."""
    return np.array(p.getMatrixFromQuaternion(quat)).reshape(3, 3)

def draw_axis(pos, orn, axis_len=0.15):
    """Draw XYZ axes at given pose."""
    R = quat_to_matrix(orn)
    p.addUserDebugLine(pos, (pos[0]+R[0,0]*axis_len, pos[1]+R[1,0]*axis_len, pos[2]+R[2,0]*axis_len),
                       [1,0,0], 2)
    p.addUserDebugLine(pos, (pos[0]+R[0,1]*axis_len, pos[1]+R[1,1]*axis_len, pos[2]+R[2,1]*axis_len),
                       [0,1,0], 2)
    p.addUserDebugLine(pos, (pos[0]+R[0,2]*axis_len, pos[1]+R[1,2]*axis_len, pos[2]+R[2,2]*axis_len),
                       [0,0,1], 2)

def main():
    # connect GUI
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    # load Panda
    robot_id = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)
    num_joints = p.getNumJoints(robot_id)

    print(f"Total joints: {num_joints}")

    # ===== user input =====
    link_index = int(input("Enter link index (-1 for base): "))
    joint_index = int(input("Enter joint index: "))

    # ----- Draw link frame origin -----
    if link_index == -1:
        pos, orn = p.getBasePositionAndOrientation(robot_id)
        link_name = "base_link"
    else:
        info = p.getJointInfo(robot_id, link_index)
        link_name = info[12].decode("utf-8")
        pos, orn = p.getLinkState(robot_id, link_index)[:2]

    draw_axis(pos, orn, axis_len=0.15)
    p.addUserDebugText(f"Link {link_index}: {link_name}",
                       [pos[0], pos[1], pos[2] + 0.1],
                       textColorRGB=[1,1,0], textSize=1.5)

    # ----- Draw joint axis -----
    if 0 <= joint_index < num_joints:
        j_info = p.getJointInfo(robot_id, joint_index)
        joint_name = j_info[1].decode("utf-8")
        joint_axis_local = np.array(j_info[13])

        parent_idx = j_info[16]
        if parent_idx == -1:
            parent_pos, parent_orn = p.getBasePositionAndOrientation(robot_id)
        else:
            parent_pos, parent_orn = p.getLinkState(robot_id, parent_idx)[:2]

        R_parent = quat_to_matrix(parent_orn)
        joint_axis_world = R_parent.dot(joint_axis_local)

        joint_pos = p.getLinkState(robot_id, joint_index)[0]
        end = joint_pos + joint_axis_world * 0.2

        p.addUserDebugLine(joint_pos, end, [1,0,1], 3)
        p.addUserDebugText(f"Joint {joint_index}: {joint_name}",
                           [end[0]+0.05, end[1]+0.05, end[2]+0.05],
                           textColorRGB=[1,0,1], textSize=1.5)
    else:
        print("Invalid joint index!")

    input("Press Enter to exit...")

if __name__ == "__main__":
    main()
