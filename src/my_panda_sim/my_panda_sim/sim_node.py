#day 4: testing ros node with simple heartbeat
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PandaSimNode(Node):
    def __init__(self):
        super().__init__('panda_sim')
        self.publisher_ = self.create_publisher(String, 'heartbeat', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Heartbeat #{self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = PandaSimNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
