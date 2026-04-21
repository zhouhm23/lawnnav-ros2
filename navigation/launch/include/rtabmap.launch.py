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
          'frame_id':'base_footprint',
          'use_sim_time':use_sim_time,
          'subscribe_rgbd':True,
          'subscribe_scan': False,
          'use_action_for_goal':True,
          'qos_scan':qos,
          'qos_image':qos,
          'qos_imu':qos,
          # RTAB-Map's parameters should be strings:
          'queue_size': 30,
          'Reg/Strategy':'1',
          'Reg/Force3DoF':'true',
          'RGBD/NeighborLinkRefining':'false',
          'Rtabmap/DetectionRate':'8',
          'RGBD/LinearUpdate':'0.10',
          'RGBD/AngularUpdate':'0.10',
          'Vis/MaxFeatures':'800',
          'OdomF2M/MaxSize':'1000',
          # Lower RangeMin so near points are not discarded (useful for close-to-robot ground)
          'Grid/RangeMin':'0.04',
          # Ensure a sensible max range for projection and grid
          'Grid/RangeMax':'3.5',
          'Grid/CellSize':'0.05',
          'Grid/MinGroundHeight':'0.20',
          'Grid/MaxGroundHeight':'0.50',
          'Grid/MapFrameProjection':'true',
          'Grid/MaxGroundAngle':'45',
          'Grid/GroundIsObstacle':'true',
          # Parameters that influence ground projection (may be ignored if node doesn't declare them)
          'proj_max_ground_height':'0.20',
          'proj_max_ground_angle':'45',
          'RGBD/ProximityBySpace':'true',
          'RGBD/ProximityPathMaxNeighbors':'5',
          'Optimizer/GravitySigma':'0', # Disable imu constraints (we are already in 2D)
          # 'Vis/CorType': '0',
          # 'OdomF2M/MaxSize': '4000',
          # 'Vis/MaxFeatures': '2000',
          # 'Optimizer/Slam2D': 'true',
          'grid_size': '20',
          'Grid/Sensor': 'true',
          'Grid/3D': 'true',
          # 'RGBD/ProximityPathMaxNeighbors': '10',
          # 'proj_max_ground_height': '0.01',
          # 'proj_max_ground_angle': '10',
    }

    remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('rgb/image', '/ascamera/camera_publisher/rgb0/image'),
            ('rgb/camera_info', '/ascamera/camera_publisher/rgb0/camera_info'),
            ('depth/image', '/ascamera/camera_publisher/depth0/image_raw'),
            ('odom', '/odom'),
            ('imu', '/imu/data'),
            ('cloud_map', '/rtabmap/cloud_map'),
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
                           'Mem/InitWMWithAllNodes': 'true' if is_localization else 'false'}]
    
    return [
        Node(
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=[{'approx_sync':True, 'approx_sync_max_interval': 0.008, 'use_sim_time':use_sim_time, 'qos':qos}],
            remappings=remappings),

        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=rtabmap_parameters,
            remappings=remappings),
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
