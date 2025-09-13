import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

import pybullet as p
import pybullet_data
import numpy as np

from my_panda_sim.fk_solver import ForwardKinematics


class FKPublisher(Node):
    def __init__(self):
        super().__init__('fk_publisher')

        self.pub = self.create_publisher(PoseStamped, '/fk_pose', 10)

        self.sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.physics_client = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)

        # Panda URDF
        self.robot_id = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

        # Joint indices (7 DOF)
        self.joint_indices = [0, 1, 2, 3, 4, 5, 6]

        # FK solver
        self.fk = ForwardKinematics(self.robot_id, self.joint_indices, ee_link_index=11)

        # save current joint
        self.current_q = None

    def joint_state_callback(self, msg: JointState):
        """when geting new joint state, publish new FK"""
        if len(msg.position) < len(self.joint_indices):
            self.get_logger().warn("JointState message has fewer positions than expected")
            return

        # first 7 joints
        self.current_q = list(msg.position[:7])

        # FK
        pos, quat = self.fk.forward(self.current_q)

        # send pos
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "panda_link0"
        pose_msg.pose.position.x = float(pos[0])
        pose_msg.pose.position.y = float(pos[1])
        pose_msg.pose.position.z = float(pos[2])
        pose_msg.pose.orientation.x = float(quat[0])
        pose_msg.pose.orientation.y = float(quat[1])
        pose_msg.pose.orientation.z = float(quat[2])
        pose_msg.pose.orientation.w = float(quat[3])

        self.pub.publish(pose_msg)
        self.get_logger().info(f"Published FK Pose from joint_states: pos={pos}, quat={quat}")


def main(args=None):
    rclpy.init(args=args)
    node = FKPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        p.disconnect()


if __name__ == '__main__':
    main()
