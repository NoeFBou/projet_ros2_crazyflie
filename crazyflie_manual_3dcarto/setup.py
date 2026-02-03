from setuptools import find_packages, setup

package_name = 'crazyflie_manual_3dcarto'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch.3dcarto_setup.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='crazyflie',
    maintainer_email='crazyflie@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
		'save_cloud = crazyflie_manual_3dcarto.save_points_cloud:main',
        	'scan_to_pcd = crazyflie_manual_3dcarto.scan_to_pcd:main',
        	'teleop_save = crazyflie_manual_3dcarto.teleop_save:main',
        ],
    },
)
