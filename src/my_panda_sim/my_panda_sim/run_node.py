import rclpy
from rclpy.node import Node
import threading

from .cube_demo import CubeDemo
from my_panda_interfaces.srv import SpawnCube

class PandaPickPlaceNode(Node):
    def __init__(self):
        super().__init__('panda_pickplace_node')

        self.demo = CubeDemo()

        sim_thread = threading.Thread(target=self.demo.run, daemon=True)
        sim_thread.start()

        self.srv = self.create_service(SpawnCube, 'spawn_cube', self.handle_spawn_cube)
        self.get_logger().info("Node ready. Service /spawn_cube available.")

    def handle_spawn_cube(self, request, response):
        cube_id = self.demo.spawn_cube()
        response.cube_id = cube_id
        return response

def main(args=None):
    rclpy.init(args=args)
    node = PandaPickPlaceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
