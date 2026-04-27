# USAGE:

# . /usr/share/gazebo/setup.sh; source /opt/ros/humble/setup.bash; source ros2_ws/install/setup.bash; ros2 launch path_coverage path_coverage.launch.py


from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取包的share目录
    package_share_directory = get_package_share_directory('path_coverage')
    
    # 构建参数文件的完整路径
    params_file_path = os.path.join(package_share_directory, 'params', 'path_coverage_params.yaml')
    
    return LaunchDescription([
        Node(
            package='path_coverage',
            executable='path_coverage_node.py',
            name='path_coverage',
            output='screen',
            parameters=[params_file_path]
        ),
    ])





