import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
import xacro

def launch_setup(context):
    compiled = os.environ['need_compile']
    sim = LaunchConfiguration('sim', default='true').perform(context)
    use_joy = LaunchConfiguration('use_joy', default='true').perform(context)
    use_depth_camera = LaunchConfiguration('use_depth_camera', default='false')
    use_lidar = LaunchConfiguration('use_lidar', default='false')
    enable_odom = LaunchConfiguration('enable_odom', default='true')
    master_name = LaunchConfiguration('master_name', default='/').perform(context)
    robot_name = LaunchConfiguration('robot_name', default='/').perform(context)
    depth_camera_name = LaunchConfiguration('depth_camera_name', default='depth_cam').perform(context)
    action_name = LaunchConfiguration('action_name', default='init').perform(context)

    sim_arg = DeclareLaunchArgument('sim', default_value=sim)
    master_name_arg = DeclareLaunchArgument('master_name', default_value=master_name)
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value=robot_name)
    depth_camera_name_arg = DeclareLaunchArgument('depth_camera_name', default_value=depth_camera_name)
    use_joy_arg = DeclareLaunchArgument('use_joy', default_value=use_joy)
    use_depth_camera_arg = DeclareLaunchArgument('use_depth_camera', default_value=use_depth_camera)
    use_lidar_arg = DeclareLaunchArgument('use_lidar', default_value=use_lidar)
    enable_odom_arg = DeclareLaunchArgument('enable_odom', default_value='true')
    action_name_arg = DeclareLaunchArgument('action_name', default_value=action_name)

    max_linear_sim = '0.7'
    max_linear = '0.2'
    max_angular_sim = '3.5'
    max_angular = '0.5'

    topic_prefix = '' if robot_name == '/' else '/%s'%robot_name
    frame_prefix = '' if robot_name == '/' else '%s/'%robot_name
    use_namespace = 'false' if robot_name == '/' else 'true'    
    namespace = '' if robot_name == '/' else robot_name
    use_sim_time = 'true' if sim == 'true' else 'false'

    map_frame = '{}map'.format(frame_prefix) if robot_name == master_name else '{}/map'.format(master_name)
    cmd_vel_topic = '{}/controller/cmd_vel'.format(topic_prefix)
    scan_raw = '{}/scan_raw'.format(topic_prefix)
    scan_topic = '{}/scan_raw'.format(topic_prefix)
    odom_frame = '{}odom'.format(frame_prefix)
    base_frame = '{}base_footprint'.format(frame_prefix)
    lidar_frame = '{}lidar_frame'.format(frame_prefix)
    imu_frame = '{}imu_link'.format(frame_prefix)

    if compiled == 'True':
        peripherals_package_path = get_package_share_directory('peripherals')
        controller_package_path = get_package_share_directory('controller')
        mentorpi_description_path = get_package_share_directory('mentorpi_description')
    else:
        peripherals_package_path = '/home/ubuntu/ros2_ws/src/peripherals'
        controller_package_path = '/home/ubuntu/ros2_ws/src/driver/controller'
        mentorpi_description_path = '/home/ubuntu/ros2_ws/src/simulations/mentorpi_description'

    xacro_file = os.path.join(mentorpi_description_path, 'urdf/mentorpi.xacro')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description_config = doc.toxml()
    robot_description = {'robot_description': robot_description_config}

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_package_path, 'launch/controller.launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': use_namespace,
            'frame_prefix': frame_prefix,
            'odom_frame': odom_frame,
            'base_frame': base_frame,
            'map_frame': map_frame,
            'imu_frame': imu_frame,
            'use_sim_time': use_sim_time,
            'enable_odom': enable_odom,
        }.items()
    )

    depth_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_package_path, 'launch/depth_camera.launch.py')),
        launch_arguments={
            'depth_camera_name': depth_camera_name,
            'tf_prefix': frame_prefix,
        }.items(),
        condition=IfCondition(use_depth_camera)
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_package_path, 'launch/lidar.launch.py')),
        launch_arguments={
            'lidar_frame': lidar_frame,
            'scan_topic': scan_topic,
            'scan_raw': scan_raw,
        }.items(),
        condition=IfCondition(use_lidar)
    )

    # rf2o_launch 已注释: LD19 精度不达标，保留仅作回滚参考
    # rf2o_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(controller_package_path, 'launch/rf2o_laser_odometry.launch.py')),
    #     condition=IfCondition(use_lidar),
    # )

    joystick_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(peripherals_package_path, 'launch/joystick_control.launch.py')),
        launch_arguments={
            'max_linear': max_linear_sim if sim == 'true' else max_linear,  
            'max_angular': max_angular_sim if sim == 'true' else max_angular,
            'remap_cmd_vel': cmd_vel_topic
        }.items(),
        condition=IfCondition(use_joy)
    )

    # init_pose_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(peripherals_package_path, 'launch/init_pose.launch.py')),
    #     launch_arguments={
    #         'action_name': action_name,
    #     }.items(),
    # )

    return [
        sim_arg, master_name_arg, robot_name_arg, depth_camera_name_arg,
        use_joy_arg, use_depth_camera_arg, use_lidar_arg, enable_odom_arg, action_name_arg,
        controller_launch,
        depth_camera_launch,
        lidar_launch,
        # rf2o_launch,  # 已注释
        joystick_control_launch,
        # init_pose_launch,
        robot_state_publisher_node,
    ]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function = launch_setup)
    ])

if __name__ == '__main__':
    # 创建一个LaunchDescription对象(create a LaunchDescription object)
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()

