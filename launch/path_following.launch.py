# Launches the control pipeline alongside LOS path following

import os 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('tsb_control')
    params_file = os.path.join(pkg_share, 'config', 'pid_params.yaml')
    control_launch = os.path.join(pkg_share, 'launch', 'control_pipeline.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(control_launch)),
        Node(
            package='tsb_control',
            executable='path_following_node',
            name='path_following_node',
            parameters=[params_file],
            output='screen',
        ),
    ])