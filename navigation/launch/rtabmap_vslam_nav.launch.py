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
import shutil
from pathlib import Path

from launch_ros.actions import PushRosNamespace, Node
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, OpaqueFunction, TimerAction, ExecuteProcess
from launch.conditions import IfCondition


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
    publish_map = LaunchConfiguration('publish_map', default='false')
    coverage_mode = LaunchConfiguration('coverage_mode', default='false')
    map_db = LaunchConfiguration('map_db', default='').perform(context)

    # 地图 db 处理
    rtabmap_db = str(Path.home() / '.ros' / 'rtabmap.db')
    if map_db:
        shutil.copy(map_db, rtabmap_db)
    else:
        if os.path.exists(rtabmap_db):
            os.remove(rtabmap_db)

    sim_arg = DeclareLaunchArgument('sim', default_value=sim)
    map_name_arg = DeclareLaunchArgument('map', default_value=map_name)
    master_name_arg = DeclareLaunchArgument('master_name', default_value=master_name)
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value=robot_name)
    localization_arg = DeclareLaunchArgument('localization', default_value=localization)
    publish_map_arg = DeclareLaunchArgument('publish_map', default_value=publish_map)
    coverage_mode_arg = DeclareLaunchArgument('coverage_mode', default_value=coverage_mode)
    map_db_arg = DeclareLaunchArgument('map_db', default_value=map_db)

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
            'enable_odom': 'false',       # 禁用出厂 EKF，用本文件专属 EKF
        }.items(),
    )

    # ── (c) 视觉+雷达专属 EKF: odom_raw + imu + rf2o(差分降权) ──
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join('/home/ubuntu/ros2_ws/src/driver/controller/config', 'ekf_vslam.yaml'),
                    {'use_sim_time': use_sim_time == 'true'}],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('odometry/filtered', 'odom'),
            ('cmd_vel', 'controller/cmd_vel'),
        ],
    )

    # 覆盖模式使用独立Nav2参数文件（三种方案共用同一coverage配置）
    nav2_params_file = os.path.join(navigation_package_path, 'config',
        'nav2_params_coverage_vslam.yaml' if coverage_mode.perform(context) == 'true' else 'nav2_params_vslam.yaml')
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_package_path, 'launch/include/bringup.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'namespace': robot_name,
            'use_namespace': use_namespace,
            'autostart': 'true',
            'rtabmap': 'true',
            'controller_param': os.path.join(navigation_package_path, 'config', 'nav2_controller_vslam.yaml'),
        }.items(),
    )

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_package_path, 'launch/include/rtabmap_vslam.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'localization': localization,
        }.items(),
    )

    bringup_launch = GroupAction(
        actions=[
            PushRosNamespace(robot_name),
            base_launch,
            ekf_node,
            TimerAction(
                period=10.0,
                actions=[navigation_launch],
            ),
            rtabmap_launch,
            TimerAction(
                period=15.0,  # 等 RTAB-Map 完全启动后发布地图
                actions=[
                    ExecuteProcess(
                        cmd=['ros2', 'service', 'call', '/rtabmap/publish_map',
                             'rtabmap_msgs/srv/PublishMap',
                             '{global_map: true, optimized: true, graph_only: false}'],
                        output='screen',
                    )
                ],
                condition=IfCondition(publish_map),
            ),
        ]
    )

    return [sim_arg, master_name_arg, robot_name_arg, localization_arg, publish_map_arg, coverage_mode_arg, map_db_arg, bringup_launch]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])


if __name__ == '__main__':
    ld = generate_launch_description()
    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
