#!/usr/bin/env python3
"""
rtabmap_vslam_nav.launch.py — (c) 视觉+雷达 RTAB-VSLAM 导航定位节点（恢复出厂）

包含:
  - rgbd_sync:       RGB+深度同步
  - rtabmap:         RTAB-Map SLAM/定位 (visual + laser ICP)
  - depth_to_scan:   pointcloud_to_laserscan 虚拟雷达（深度相机→LaserScan→Nav2代价图）

相机 topic: 出厂原始 /ascamera/camera_publisher/...（未经 remap）
"""

from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, OpaqueFunction
import os
from ament_index_python.packages import get_package_share_directory


def launch_setup(context):
    use_sim_time = LaunchConfiguration('use_sim_time')
    qos = LaunchConfiguration('qos')
    localization = LaunchConfiguration('localization').perform(context)

    parameters = {
        'frame_id': 'base_footprint',
        'use_sim_time': use_sim_time,
        'use_action_for_goal': True,
        'qos_scan': qos,
        'qos_image': qos,
        'qos_imu': qos,
    }

    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
        ('rgb/image', '/ascamera/camera_publisher/rgb0/image'),
        ('rgb/camera_info', '/ascamera/camera_publisher/rgb0/camera_info'),
        ('depth/image', '/ascamera/camera_publisher/depth0/image_raw'),
        ('scan', '/scan_raw'),
        ('grid_map', '/map'),
        ('odom', '/odom'),
        ('imu', '/imu/data'),
        ('cloud_map', '/rtabmap/cloud_map'),
        ('cloud_obstacles', '/rtabmap/cloud_obstacles'),
    ]

    nav_share = get_package_share_directory('navigation')
    rtabmap_params_file = os.path.join(nav_share, 'config', 'rtabmap_params_vslam.yaml')

    is_localization = (localization == 'true')

    rtabmap_parameters = [parameters, rtabmap_params_file,
                          {'Mem/IncrementalMemory': 'false' if is_localization else 'true',
                           'Mem/InitWMWithAllNodes': 'true' if is_localization else 'false',
                           'RGBD/StartAtOrigin': 'false'}]

    return [
        Node(
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=[{'approx_sync': True, 'approx_sync_max_interval': 0.05,
                         'use_sim_time': use_sim_time, 'qos': qos}],
            remappings=remappings),

        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=rtabmap_parameters,
            remappings=remappings),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'qos', default_value='2',
            description='QoS used for input sensor topics'),

        DeclareLaunchArgument(
            'localization', default_value='false',
            description='Launch in localization mode.'),

        OpaqueFunction(function=launch_setup)
    ])


if __name__ == '__main__':
    ld = generate_launch_description()
    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
