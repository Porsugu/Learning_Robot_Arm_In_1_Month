import rclpy
from rclpy.node import Node
import threading

from .cube_demo import CubeDemo
from my_panda_interfaces.srv import SpawnCube, SetAutorun, GetStatus, SetDropPos
from my_panda_interfaces.msg import TaskStatus


class PandaPickPlaceNode(Node):
    """ROS2 node wrapping CubeDemo with services for task control."""

    def __init__(self):
        super().__init__('panda_pickplace_node')

        # Initialize CubeDemo with node reference
        self.demo = CubeDemo(self)

        # Bind publisher for task status updates
        self.demo.status_pub = self.create_publisher(TaskStatus, 'task_status', 10)

        # Start background simulation thread
        sim_thread = threading.Thread(target=self.demo.run, daemon=True)
        sim_thread.start()

        # Define services
        self.create_service(SpawnCube, 'spawn_cube', self.handle_spawn_cube)
        self.create_service(SetAutorun, 'set_autorun', self.handle_set_autorun)
        self.create_service(GetStatus, 'get_status', self.handle_get_status)
        self.create_service(SetDropPos, 'set_drop_pos', self.handle_set_drop_pos)

        self.get_logger().info(
            "✅ PandaPickPlaceNode ready. Services: "
            "/spawn_cube, /set_autorun, /get_status, /set_drop_pos"
        )

    # ---------------- Service callbacks ----------------
    def handle_spawn_cube(self, request, response):
        cube_id = self.demo.spawn_cube()
        response.cube_id = cube_id
        return response

    def handle_set_autorun(self, request, response):
        ok = self.demo.set_autorun(request.enable)
        response.ok = bool(ok)
        return response

    def handle_get_status(self, request, response):
        gen, comp, pend = self.demo.get_status()
        response.total_generated = gen
        response.total_completed = comp
        response.pending = pend
        return response

    def handle_set_drop_pos(self, request, response):
        ok = self.demo.set_drop_pos(request.x, request.y, request.z)
        response.ok = bool(ok)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PandaPickPlaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
