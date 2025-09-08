# ros2_ws — Week 1 (Day4–7)

## 📌 Overview
This project is part of a 30-day robotics programming learning plan.  
During **Week 1 (Day1–7)** the goal was to set up the environment, run PyBullet simulations, and publish robot joint states to ROS2.

## ✅ Achievements (Day1–7)
- Installed Ubuntu 22.04 with **ROS2 Humble** and **Python3 environment**.
- Installed **PyBullet** and successfully loaded the Franka Panda URDF.
- Created a ROS2 node `joint_state_publisher`:
  - Connects to PyBullet (GUI or DIRECT mode).
  - Discovers movable joints (revolute and prismatic).
  - Reads joint positions, velocities, and efforts from PyBullet.
  - Publishes them to the standard ROS2 topic `/joint_states`.
- Verified the output with:
  - `ros2 topic echo /joint_states`
  - (optional) PyBullet GUI visualization.
- Added a **demo mode** where the first joint performs a sinusoidal motion.

## 🚀 How to Run

### 1. Terminal A — Run the ROS2 node
```bash
cd ~/ros2_ws/src/my_panda_sim/my_panda_sim
source /opt/ros/humble/setup.bash
python3 joint_state_publisher.py --direct --demo
```
### 2. Terminal B — Echo the topic
```
source /opt/ros/humble/setup.bash
ros2 topic list
ros2 topic echo /joint_states
```
You should see continuous joint state messages with arrays of name, position, velocity, and effort.

### ⚙️ Command-line Arguments
```
Argument	Default	Description
--gui	off	Run PyBullet in GUI mode (opens a 3D window).
--direct	on (default if not --gui)	Run PyBullet in DIRECT/headless mode (faster, no window).
--rate N	50.0 Hz	Publish rate in Hertz (e.g., --rate 10 for 10Hz).
--urdf PATH	franka_panda/panda.urdf	Path to a URDF file to load (default: Franka Panda).
--include-fingers	off	If set, also publish the gripper/finger joints.
--demo	off	Apply sinusoidal motion to the first joint (useful for testing and demos).
```

### Examples:

# Run with GUI visualization and sinusoidal demo
python3 joint_state_publisher.py --gui --demo

# Run at 10Hz in headless mode
python3 joint_state_publisher.py --direct --rate 10

# Run with a custom URDF
python3 joint_state_publisher.py --urdf ~/robots/ur5.urdf --direct

###🎥 Demo Screenshot
![Recording 2025-09-07 215054](https://github.com/user-attachments/assets/929a7fd0-be34-4198-a5bd-d5590304cfca)

### 📂 Repository Structure
```
my_panda_sim/
├── README.md
└── my_panda_sim/
    └── joint_state_publisher.py
```
### 🔮 Next Steps (Week2)

Extract DH parameters from the URDF.

Implement Forward Kinematics (FK) using DH or homogeneous transformation matrices.

Compare FK results with PyBullet’s built-in forward kinematics.

Begin experimenting with Inverse Kinematics (IK).

### Project achievement
💡 This Week 1 marks the foundation of the robotics programming journey:
setting up the environment, understanding URDFs, connecting PyBullet with ROS2, and publishing standard joint states.



