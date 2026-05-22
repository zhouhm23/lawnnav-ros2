from launch_ros.actions import Node
from launch import LaunchDescription
from launch import LaunchService
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, OpaqueFunction
import os
from ament_index_python.packages import get_package_share_directory

def launch_setup(context):
    use_sim_time = LaunchConfiguration('use_sim_time')
    qos = LaunchConfiguration('qos')
    localization = LaunchConfiguration('localization').perform(context)

    parameters={
          # === Launch-level params (ROS topics, QoS, frame) ===
          'frame_id':'base_footprint',
          'use_sim_time':use_sim_time,
          'use_action_for_goal':True,
          'qos_scan':qos,
          'qos_image':qos,
          'qos_imu':qos,
          # === All RTAB-Map algorithm params are in rtabmap_params.yaml ===
    }

    remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('rgb/image', '/ascamera/camera_publisher/rgb0/image'),
            ('rgb/camera_info', '/ascamera/camera_publisher/rgb0/camera_info'),
            ('depth/image', '/ascamera/camera_publisher/depth0/image_raw'),
            ('scan', '/scan_raw'),  # 虚拟雷达→RTAB-Map ICP定位匹配
            ('grid_map', '/map'),  # RTAB-Map占据栅格（比pgm/yaml保真度高）
            ('odom', '/odom'),
            ('imu', '/imu/data'),
            ('cloud_map', '/rtabmap/cloud_map'),
            ('cloud_obstacles', '/rtabmap/cloud_obstacles'),
          ]

    # path to optional params file in navigation package
    nav_share = get_package_share_directory('navigation')
    rtabmap_params_file = os.path.join(nav_share, 'config', 'rtabmap_params.yaml')

    # Logic for localization vs mapping
    # Localization: Mem/IncrementalMemory=false, Mem/InitWMWithAllNodes=true
    # Mapping: Mem/IncrementalMemory=true, Mem/InitWMWithAllNodes=false
    
    is_localization = (localization == 'true')
    
    rtabmap_parameters = [parameters, rtabmap_params_file, 
                          {'Mem/IncrementalMemory': 'false' if is_localization else 'true',
                           'Mem/InitWMWithAllNodes': 'true' if is_localization else 'false',
                           'RGBD/StartAtOrigin': 'false'}]
    
    return [
        Node(
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=[{'approx_sync':True, 'approx_sync_max_interval': 0.05, 'use_sim_time':use_sim_time, 'qos':qos}],
            remappings=remappings),

        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=rtabmap_parameters,
            remappings=remappings),

        # === 虚拟雷达节点：深度相机点云 → LaserScan → /scan_raw ===
        # 原理：提取点云中离地2cm-30cm的点投影到水平面，生成标准LaserScan。
        # Nav2代价图和RTAB-Map都读/scan_raw，它们不知道底层是深度相机。
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            name='depth_to_scan',
            output='screen',
            parameters=[os.path.join(nav_share, 'config', 'pointcloud_to_laserscan_params.yaml'),
                        {'use_sim_time': use_sim_time}],
            remappings=[
                ('cloud_in', '/ascamera/camera_publisher/depth0/points'),
                ('scan', '/scan_raw'),
            ]),
    ]

def generate_launch_description():
    return LaunchDescription([

        # Launch arguments
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
    # 创建一个LaunchDescription对象(create a LaunchDescription object)
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
