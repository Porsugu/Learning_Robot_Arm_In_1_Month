from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_panda_sim',
            executable='run_node',
            name='panda_pickplace_node'
        )
    ])
