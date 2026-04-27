from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory('coverage_evaluator')
    params_file = os.path.join(share, 'params', 'coverage_evaluator_params.yaml')

    return LaunchDescription([
        Node(
            package='coverage_evaluator',
            executable='coverage_evaluator_node',
            name='coverage_evaluator',
            output='screen',
            parameters=[params_file],
        ),
    ])
