from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'tsb_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Laura Martinho',
    maintainer_email='lauramartinhof@gmail.com',
    description='Surge and heading PID control with thruster allocation.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_node = tsb_control.sensor_node:main',
            'pid_node = tsb_control.pid_node:main',
            'allocation_node = tsb_control.allocation_node:main',
            'path_follower_node = tsb_control.path_follower_node:main',
        ],
    },
)
