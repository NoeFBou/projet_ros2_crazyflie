from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pointcloud_builder'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='todo@todo.todo',
    description='Package setup corrected',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'cloud_assembler = pointcloud_builder.cloud_assembler_node:main',
            'PlyPublisher = pointcloud_builder.PlyPublisher:main',
            'CustomAMCL = pointcloud_builder.CustomAMCL:main',
            'planner = pointcloud_builder.planner:main',
            'goal_manager = pointcloud_builder.goal_manager:main',
        ],
    },
)