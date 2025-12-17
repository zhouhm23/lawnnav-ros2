from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os


def generate_launch_description():
    # Compute script path relative to this launch file so it works from source workspace
    this_dir = os.path.dirname(__file__)
    script = os.path.abspath(os.path.join(this_dir, '..', 'scripts', 'map_color_overlay.py'))

    return LaunchDescription([
        ExecuteProcess(
            # Use 'python3' on systems where 'python' is not available
            cmd=['python3', script],
            output='screen'
        )
    ])


if __name__ == '__main__':
    generate_launch_description()
