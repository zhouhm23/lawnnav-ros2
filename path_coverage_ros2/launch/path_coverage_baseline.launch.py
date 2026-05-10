# BASELINE launch — 对照组：原始 path_coverage，无鲁棒性改进
# ros2 launch path_coverage path_coverage_baseline.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_share_directory = get_package_share_directory('path_coverage')
    params_file_path = os.path.join(package_share_directory, 'params', 'path_coverage_baseline_params.yaml')
    return LaunchDescription([
        Node(
            package='path_coverage',
            executable='path_coverage_node_baseline.py',
            name='path_coverage',
            output='screen',
            parameters=[params_file_path]
        ),
    ])
