from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'navigation3d'

def package_files(data_files, directory_list):
    paths_dict = {}
    for directory in directory_list:
        for (path, directories, filenames) in os.walk(directory):
            for filename in filenames:
                file_path = os.path.join(path, filename)
                install_path = os.path.join('share', package_name, path)
                if install_path in paths_dict:
                    paths_dict[install_path].append(file_path)
                else:
                    paths_dict[install_path] = [file_path]
    for key in paths_dict:
        data_files.append((key, paths_dict[key]))
    return data_files
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    (os.path.join('share', package_name, 'config'), glob('config/*')),
    (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
    (os.path.join('share', package_name, 'maps'), glob('maps/*.bt')),
]
package_files(data_files, ['meshes'])
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='todo@todo.todo',
    description='Package setup corrected',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'planner = navigation3d.planner:main',
            'trajectories_follower = navigation3d.trajectories_follower:main',
            'supervisor = navigation3d.supervisor:main',
            'interactive_marker_pose_stamped = navigation3d.interactive_marker_pose_stamped:main',
        ],
    },
)