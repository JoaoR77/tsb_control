from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tsb_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='joao_r',
    maintainer_email='joao.rasoilo@tecnico.ulisboa.pt',
    description='2-DOF PID control, allocation, and LOS path following for TSB recruitment',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sensor_processing_node = tsb_control.sensor_processing_node:main',
            'pid_node = tsb_control.pid_node:main',
            'allocation_node = tsb_control.allocation_node:main',
            'path_following_node = tsb_control.path_following_node:main',
        ],
    },
)
