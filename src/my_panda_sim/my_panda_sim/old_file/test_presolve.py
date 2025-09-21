import pybullet as p
import pybullet_data
import numpy as np
import time

def move_joints_smoothly(robot_id, q_start, q_end, step_size=0.02, sleep_time=0.01):
    """
    平滑地從 q_start 移動到 q_end
    """
    q_start = np.array(q_start, dtype=float)
    q_end = np.array(q_end, dtype=float)

    diff = q_end - q_start
    n_steps = int(np.max(np.abs(diff)) / step_size) + 1

    for alpha in np.linspace(0, 1, n_steps):
        q_interp = (1 - alpha) * q_start + alpha * q_end
        for j in range(len(q_interp)):
            p.resetJointState(robot_id, j, float(q_interp[j]), targetVelocity=0.0)
        p.stepSimulation()
        time.sleep(sleep_time)

def main():
    # 初始化環境
    cid = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    # 載入 Panda
    robot_id = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)
    dof = 7

    # 定義 home pose
    home_pose = [0.0, -np.pi/4, 0.0, -np.pi/2, 0.0, np.pi/2, 0.0]

    print("Moving to home pose...")
    move_joints_smoothly(robot_id, [0.0]*dof, home_pose)
    time.sleep(1)

    # 目標底座角度（例如 -90 度）
    target_base_angle = -np.pi

    # 保持其他 joint 不變，只旋轉 joint0
    q_new = home_pose.copy()
    q_new[0] = target_base_angle

    print(f"Rotating base to {np.degrees(target_base_angle):.1f}°...")
    move_joints_smoothly(robot_id, home_pose, q_new)
    time.sleep(1)

    # 保持姿態
    while True:
        p.stepSimulation()
        time.sleep(1./240.)

if __name__ == "__main__":
    main()
