# DEPRECATED 原版恢复: 此文件内容回退自 git commit e39c77e (重构前最后正常版本)
# 仅修改 2 处: include 路径 + use_depth_camera flag
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
    map_db_arg = DeclareLaunchArgument('map_db', default_value=map_db)

    use_sim_time = 'true' if sim == 'true' else 'false'
    use_namespace = 'true' if robot_name != '/' else 'false'
    frame_prefix = '' if robot_name == '/' else '%s/'%robot_name
    topic_prefix = '' if robot_name == '/' else '/%s'%robot_name
    map_frame = '{}map'.format(frame_prefix)
    odom_frame = '{}odom'.format(frame_prefix)
    base_frame = '{}base_footprint'.format(frame_prefix)
    depth_camera_topic = '/ascamera/camera_publisher/depth0/image_raw'.format(topic_prefix)
    depth_camera_info = '/ascamera/camera_publisher/rgb0/camera_info'.format(topic_prefix)
    rgb_camera_topic = '/ascamera/camera_publisher/rgb0/image'.format(topic_prefix)
    odom_topic = '{}/odom'.format(topic_prefix)
    scan_topic = '{}/scan_raw'.format(topic_prefix)

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_package_path, 'launch/include/robot.launch.py')),
        launch_arguments={
            'sim': sim,
            'master_name': master_name,
            'robot_name': robot_name,
            'action_name': 'horizontal',
            'use_depth_camera': 'true',   # 适配新 robot.launch.py 双 bool
            'enable_odom': 'false',       # 禁用 controller 内置 EKF，用本文件专属 EKF
        }.items(),
    )

    # ── (a) 单相机专属 EKF: pose 差分模式，修正系数生效 ──
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join('/home/ubuntu/ros2_ws/src/driver/controller/config', 'ekf_camera.yaml'),
                    {'use_sim_time': use_sim_time == 'true'}],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('odometry/filtered', 'odom'),
            ('cmd_vel', 'controller/cmd_vel'),
        ],
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(navigation_package_path, 'launch/include/bringup.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': os.path.join(navigation_package_path, 'config', 'nav2_params.yaml'),
            'namespace': robot_name,
            'use_namespace': use_namespace,
            'autostart': 'true',
            'rtabmap': 'true',
            'controller_param': os.path.join(navigation_package_path, 'config', 'nav2_controller_camera.yaml'),
        }.items(),
    )

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_package_path, 'launch/include/rtabmap_camera.launch.py')),  # 改为新文件名
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

    return [sim_arg, master_name_arg, robot_name_arg, localization_arg, publish_map_arg, map_db_arg, bringup_launch]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function = launch_setup)
    ])

if __name__ == '__main__':
    ld = generate_launch_description()
    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
