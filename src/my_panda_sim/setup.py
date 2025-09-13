from setuptools import setup
import os
from glob import glob

package_name = 'my_panda_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arthur',
    maintainer_email='arthur@example.com',
    description='My Panda Simulation (FK + IK tools)',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'fk_publisher = my_panda_sim.fk_publisher:main',
        ],
    },
)
