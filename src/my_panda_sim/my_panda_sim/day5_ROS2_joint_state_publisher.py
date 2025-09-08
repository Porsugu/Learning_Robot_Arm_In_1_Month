import rclpy
from  rclpy.node import Node
from sensor_msgs.msg import JointState

import pybullet as p
import pybullet_data
import sys

import time, math
import argparse

class JointStatePublisherNode(Node):
    def __init__(self, args):
        super().__init__('joint_state_publisher')
        self.pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        #1) connect to pybullet
        if args.gui and not args.direct:
            mode = p.GUI
        else:
            mode = p.DIRECT
        self.client_id = p.connect(mode)
        if self.client_id < 0:
            self.get_logger().error('Failed to connect to robot.')
            sys.exit(1)

        #2) find URDF
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)

        #3)load Franka Panda
        self.robot_id = p.loadURDF('franka_panda/panda.urdf', useFixedBase=True)
        self.get_logger().info(f'Loaded robot id={self.robot_id}')

        #4) discover movable joints
        self.SUPPORTED = {p.JOINT_REVOLUTE, p.JOINT_PRISMATIC}
        self.joint_indices, self.joint_names = self.discover_movable_joints()
        for i, n in zip(self.joint_indices, self.joint_names):
            self.get_logger().info(f"Publish joint -> idx={i}, name={n}")


        #6) swing the arm
        self.include_fingers = args.include_fingers
        self.demo_motion = args.demo
        self.t0 = time.time()
        self.max_force = 50


        # 5) timer
        self.timer = self.create_timer(1.0 / args.rate, self.on_timer)



    def discover_movable_joints(self):
        indices, names = [],[]
        for j in range(p.getNumJoints(self.robot_id)):
            info = p.getJointInfo(self.robot_id,j)
            jtype = info[2]
            name = info[1].decode('utf-8')
            if jtype in self.SUPPORTED:
                indices.append(j)
                names.append(name)
        return indices, names

    def on_timer(self):

        # 6) swing the arm
        if self.joint_indices:
            t = time.time() - self.t0
            target = 0.5 * math.sin(2.0 * math.pi * 0.2 * t)  # 0.5rad, 0.2Hz
            j0 = self.joint_indices[0]
            p.setJointMotorControl2(self.robot_id, j0, p.POSITION_CONTROL,
                                    targetPosition=target, force=self.max_force)

        msg = JointState()                      #make a string
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = []
        msg.position = []
        msg.velocity = []
        msg.effort = []

        positions, velocities, efforts = [], [], []
        for j in self.joint_indices:
            pos,val,_,torque = p.getJointState(self.robot_id,j)
            positions.append(float(pos))
            velocities.append(float(val))
            efforts.append(float(torque))

        msg.name = list(self.joint_names)
        msg.position = positions
        msg.velocity = velocities
        msg.effort = efforts
        self.pub.publish(msg)        # publish the message
        p.stepSimulation()

def build_argparser():
    ap = argparse.ArgumentParser(description="Publish /joint_states from PyBullet")
    ap.add_argument('--gui', action='store_true', help='Use PyBullet GUI window')
    ap.add_argument('--direct', action='store_true', help='Use PyBullet DIRECT (headless)')
    ap.add_argument('--rate', type=float, default=50.0, help='Publish rate in Hz (default=50)')
    ap.add_argument('--urdf', type=str, default='franka_panda/panda.urdf',
                    help='Path to URDF file')
    ap.add_argument('--include-fingers', action='store_true',
                    help='Include gripper/finger joints')
    ap.add_argument('--demo', action='store_true',
                    help='Apply sinusoidal motion to first joint')
    return ap

def main():
    rclpy.init()
    parser = build_argparser()
    args = parser.parse_args()
    node = JointStatePublisherNode(args)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
