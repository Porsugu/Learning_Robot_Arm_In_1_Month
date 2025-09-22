from setuptools import setup

package_name = 'my_panda_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/run.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arthur',
    maintainer_email='arthur@todo.todo',
    description='Panda pick and place simulation with ROS2 + PyBullet',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'run_node = my_panda_sim.run_node:main',
        ],
    },
)
