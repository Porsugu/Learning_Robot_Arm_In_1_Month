import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLabel
from PyQt5.QtCore import QTimer
import rclpy
from rclpy.node import Node

from my_panda_interfaces.srv import SpawnCube, SetAutorun
from my_panda_interfaces.msg import TaskStatus


class PandaControlGUI(QWidget):
    def __init__(self, node: Node):
        super().__init__()
        self.node = node

        self.setWindowTitle("Panda Pick & Place Control")
        self.setGeometry(200, 200, 300, 200)

        layout = QVBoxLayout()

        # mission status
        self.label_generated = QLabel("Generated: 0")
        self.label_completed = QLabel("Completed: 0")
        self.label_pending = QLabel("Pending: 0")
        layout.addWidget(self.label_generated)
        layout.addWidget(self.label_completed)
        layout.addWidget(self.label_pending)

        # btn
        btn_spawn = QPushButton("Spawn Cube")
        btn_spawn.clicked.connect(self.on_spawn_cube)
        layout.addWidget(btn_spawn)

        btn_pause = QPushButton("Pause Autorun")
        btn_pause.clicked.connect(lambda: self.on_set_autorun(False))
        layout.addWidget(btn_pause)

        btn_resume = QPushButton("Resume Autorun")
        btn_resume.clicked.connect(lambda: self.on_set_autorun(True))
        layout.addWidget(btn_resume)

        self.setLayout(layout)

        # 用 timer 定期呼叫 ROS spin_once
        self.timer = QTimer()
        self.timer.timeout.connect(self.ros_spin)
        self.timer.start(100)  # 每 100ms

    def ros_spin(self):
        rclpy.spin_once(self.node, timeout_sec=0)

    def on_spawn_cube(self):
        req = SpawnCube.Request()
        future = self.node.cli_spawn.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result():
            cube_id = future.result().cube_id
            print(f"[GUI] Spawned cube id={cube_id}")

    def on_set_autorun(self, enable: bool):
        req = SetAutorun.Request()
        req.enable = enable
        future = self.node.cli_autorun.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result():
            state = "enabled" if enable else "paused"
            print(f"[GUI] Autorun {state}")


class PandaControlNode(Node):
    def __init__(self):
        super().__init__('panda_control_gui_node')

        # service clients
        self.cli_spawn = self.create_client(SpawnCube, 'spawn_cube')
        self.cli_autorun = self.create_client(SetAutorun, 'set_autorun')

        self.sub_status = self.create_subscription(TaskStatus, 'task_status', self.status_callback, 10)

        self.gui = None

    def status_callback(self, msg: TaskStatus):
        if self.gui:
            self.gui.label_generated.setText(f"Generated: {msg.total_generated}")
            self.gui.label_completed.setText(f"Completed: {msg.total_completed}")
            self.gui.label_pending.setText(f"Pending: {msg.pending}")


def main(args=None):
    rclpy.init(args=args)
    node = PandaControlNode()

    app = QApplication(sys.argv)
    gui = PandaControlGUI(node)
    node.gui = gui

    gui.show()
    app.exec_()

    node.destroy_node()
    rclpy.shutdown()
