# Launches the 3-node 2-DOF control pipeline with YAML parameters

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('tsb_control')
    params_file = os.path.join(pkg_share, 'config', 'pid_params.yaml')

    return LaunchDescription([
        Node(
            package='tsb_control',
            executable='sensor_processing_node',
            name='sensor_processing_node',
            output='screen',
        ),

        Node(
            package='tsb_control',
            executable='pid_node',
            name='pid_node',
            parameters=[params_file],
            output='screen',
        ),

        Node(
            package='tsb_control',
            executable='allocation_node',
            name='allocation_node',
            parameters=[params_file],
            output='screen',
        ),
    ])