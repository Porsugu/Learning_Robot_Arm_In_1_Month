import pybullet as p
import pybullet_data
import time
import numpy as np

# --- PyBullet 初始化 ---
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)  # 關閉不必要的面板

# 載入地面和機器人
p.loadURDF("plane.urdf")
robot_id = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

# 獲取關節數量
num_joints = p.getNumJoints(robot_id)
print(f"Robot has {num_joints} joints.")

# --- 測試開始 ---
print("\n--- Starting Joint 0 Rotation Test ---")

# 定義要測試的角度 (弧度) 和對應的名稱
angles_to_test = {
    "0 degrees (East, +X)": 0.0,
    "90 degrees (North, +Y)": np.pi / 2,
    "180 degrees (West, -X)": np.pi,
    "270 degrees (South, -Y)": 3 * np.pi / 2
}

try:
    for name, angle in angles_to_test.items():
        print(f"\nSetting joint[0] to: {name} ({angle:.4f} radians)")

        # 使用 resetJointState 直接、無物理地設定角度
        # 這能確保我們只測試角度本身，不受物理引擎干擾
        p.resetJointState(
            bodyUniqueId=robot_id,
            jointIndex=0,  # 只控制 joint 0
            targetValue=angle  # 設定目標角度
        )

        # 保持姿勢並讓模擬運行
        for _ in range(240 * 2):  # 停留 2 秒
            p.stepSimulation()
            time.sleep(1. / 240.)

    print("\n--- Test Finished ---")

    # 讓視窗保持開啟
    while True:
        p.stepSimulation()
        time.sleep(1. / 240.)

except KeyboardInterrupt:
    p.disconnect()