from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # robot_state_publisher
    panda_desc = get_package_share_directory('moveit_resources_panda_description')
    urdf_file = os.path.join(panda_desc, 'urdf', 'panda.urdf')
    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'panda_link0']
    )

    fk_publisher = Node(
        package='my_panda_sim',
        executable='fk_publisher',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        static_tf,
        fk_publisher
    ])
