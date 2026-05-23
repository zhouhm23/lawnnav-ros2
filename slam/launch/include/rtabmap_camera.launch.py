from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch import LaunchService
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    qos = LaunchConfiguration('qos')

    parameters = {
        'frame_id': 'base_footprint',
        'use_sim_time': use_sim_time,
        'subscribe_rgbd': True,
        'subscribe_scan': True,
        'use_action_for_goal': True,
        'qos_scan': qos,
        'qos_image': qos,
        'qos_imu': qos,
        'queue_size': 50,  
        'Reg/Strategy': '1',
        'Reg/Force3DoF': 'true',
        # Reduce RangeMin so near points are not discarded
        'Grid/RangeMin': '0.02',
        'Grid/RangeMax': '5.0',
        'Grid/CellSize': '0.05',
        # Projection/ground related parameters
        'proj_max_ground_height': '0.20',
        'proj_max_ground_angle': '20',
        'RGBD/ProximityBySpace': 'true',
        'RGBD/ProximityPathMaxNeighbors': '10',
        'Optimizer/GravitySigma': '0',  # Disable imu constraints (we are already in 2D)
        'Grid/Sensor': 'true',
        #'approx_sync_max_interval': 0.02,  
        #'queue_size_imu': 300,  
    }

    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
        ('rgb/image', '/ascamera/camera_publisher/rgb0/image'),
        ('rgb/camera_info', '/ascamera/camera_publisher/rgb0/camera_info'),
        ('depth/image', '/ascamera/camera_publisher/depth0/image_raw'),
        ('odom', '/odom'),
        ('scan','/scan_raw'),
    ]

    # load params YAML from slam package config if available
    slam_share = get_package_share_directory('slam')
    rtabmap_params_file = os.path.join(slam_share, 'config', 'rtabmap_params.yaml')

    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'qos', default_value='2',
            description='QoS used for input sensor topics'),

        # Nodes to launch
        Node(
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=[{'approx_sync': True, 'approx_sync_max_interval': 0.05, 'use_sim_time': use_sim_time, 'qos': qos}],
            remappings=remappings),

        # SLAM Mode:
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters, rtabmap_params_file],
            remappings=remappings,
            arguments=['-d']),
    ])

if __name__ == '__main__':
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()



