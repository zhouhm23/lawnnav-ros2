#!/usr/bin/env python3
"""
rtabmap_vslam_nav.launch.py — (c) 视觉+雷达 RTAB-VSLAM 导航/覆盖（恢复出厂）

定位方式: RTAB-Map localization (visual + laser ICP), 跳过 AMCL
代价图输入: /scan_raw (真实激光雷达 + pointcloud_to_laserscan 虚拟雷达)

TF 树:
  map → odom → base_footprint → base_link
                                → lidar_frame
                                → depth_cam → depth_cam_optical

用法:
    ros2 launch navigation rtabmap_vslam_nav.launch.py localization:=false   # 建图
    ros2 launch navigation rtabmap_vslam_nav.launch.py localization:=true    # 纯定位
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import PushRosNamespace
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, OpaqueFunction, TimerAction


def launch_setup(context):
    compiled = os.environ['need_compile']
    if compiled == 'True':
        slam_package_path = get_package_share_directory('slam')
        navigation_package_path = get_package_share_directory('navigation')
    else:
        slam_package_path = '/home/ubuntu/ros2_ws/src/slam'
        navigation_package_path = '/home/ubuntu/ros2_ws/src/navigation'

    sim = LaunchConfiguration('sim', default='false').perform(context)
    map_name = LaunchConfiguration('map', default='').perform(context)
    robot_name = LaunchConfiguration('robot_name', default=os.environ['HOST']).perform(context)
    master_name = LaunchConfiguration('master_name', default=os.environ['MASTER']).perform(context)
    localization = LaunchConfiguration('localization', default='false').perform(context)

    sim_arg = DeclareLaunchArgument('sim', default_value=sim)
    map_name_arg = DeclareLaunchArgument('map', default_value=map_name)
    master_name_arg = DeclareLaunchArgument('master_name', default_value=master_name)
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value=robot_name)
    localization_arg = DeclareLaunchArgument('localization', default_value=localization)

    use_sim_time = 'true' if sim == 'true' else 'false'
    use_namespace = 'true' if robot_name != '/' else 'false'
    frame_prefix = '' if robot_name == '/' else '%s/' % robot_name
    topic_prefix = '' if robot_name == '/' else '/%s' % robot_name

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_package_path, 'launch/include/robot.launch.py')),
        launch_arguments={
            'sim': sim,
            'master_name': master_name,
            'robot_name': robot_name,
            'use_depth_camera': 'true',
            'use_lidar': 'true',
            'action_name': 'horizontal',
        }.items(),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_package_path, 'launch/include/bringup.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': os.path.join(navigation_package_path, 'config', 'nav2_params_vslam.yaml'),
            'namespace': robot_name,
            'use_namespace': use_namespace,
            'autostart': 'true',
            'rtabmap': 'true',
        }.items(),
    )

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_package_path, 'launch/include/rtabmap_vslam_nav.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'localization': localization,
        }.items(),
    )

    bringup_launch = GroupAction(
        actions=[
            PushRosNamespace(robot_name),
            base_launch,
            TimerAction(
                period=10.0,
                actions=[navigation_launch],
            ),
            rtabmap_launch
        ]
    )

    return [sim_arg, master_name_arg, robot_name_arg, localization_arg, bringup_launch]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])


if __name__ == '__main__':
    ld = generate_launch_description()
    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
