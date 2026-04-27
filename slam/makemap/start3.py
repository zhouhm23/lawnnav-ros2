#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
import pyrealsense2 as rs
import numpy as np
from cv_bridge import CvBridge
import tf2_ros
import time
import copy

class D43515HzNode(Node):
    def __init__(self):
        super().__init__('d435_15hz_node')
        
        self.get_logger().info('正在初始化D435节点...')
        
        # 创建发布者
        self.color_pub = self.create_publisher(Image, '/rgb/image', 10)
        self.depth_pub = self.create_publisher(Image, '/depth/image', 10)
        self.color_info_pub = self.create_publisher(CameraInfo, '/rgb/camera_info', 10)
        # 兼容旧系统的 ascamera 话题（同时发布，便于平滑切换）
        self.asc_color_pub = self.create_publisher(Image, '/ascamera/camera_publisher/rgb0/image', 10)
        self.asc_depth_pub = self.create_publisher(Image, '/ascamera/camera_publisher/depth0/image_raw', 10)
        self.asc_color_info_pub = self.create_publisher(CameraInfo, '/ascamera/camera_publisher/rgb0/camera_info', 10)
        
        # 创建TF广播器
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        self.bridge = CvBridge()
        
        # 初始化相机（降低分辨率）
        self.init_camera_low_res()
        
        # 创建定时器 - 15Hz
        self.timer = self.create_timer(0.067, self.publish_data)  # ~15Hz
        
        self.get_logger().info('D435节点初始化完成，分辨率：424x240')

    def init_camera_low_res(self):
        """初始化相机 - 降低分辨率以提高性能"""
        try:
            self.pipeline = rs.pipeline()
            config = rs.config()
            
            # 使用424x240分辨率，15Hz帧率
            config.enable_stream(rs.stream.color, 424, 240, rs.format.bgr8, 15)
            config.enable_stream(rs.stream.depth, 424, 240, rs.format.z16, 15)
            
            # 启动管道
            self.pipeline_profile = self.pipeline.start(config)
            
            # 获取相机内参
            self.get_camera_intrinsics()
            
            self.get_logger().info('相机配置为424x240分辨率，15Hz帧率')
            
        except Exception as e:
            self.get_logger().error(f'初始化相机失败: {str(e)}')
            # 尝试备用配置
            self.try_fallback_config()

    def try_fallback_config(self):
        """尝试备用配置"""
        try:
            self.get_logger().info('尝试备用配置...')
            config = rs.config()
            
            # 备用配置：640x360分辨率
            config.enable_stream(rs.stream.color, 640, 360, rs.format.bgr8, 15)
            config.enable_stream(rs.stream.depth, 640, 360, rs.format.z16, 15)
            
            self.pipeline_profile = self.pipeline.start(config)
            self.get_camera_intrinsics()
            self.get_logger().info('相机配置为640x360分辨率，15Hz帧率')
            
        except Exception as e:
            self.get_logger().error(f'备用配置也失败: {str(e)}')
            raise

    def get_camera_intrinsics(self):
        """获取相机内参"""
        try:
            color_profile = self.pipeline_profile.get_stream(rs.stream.color)
            color_intr = color_profile.as_video_stream_profile().get_intrinsics()
            
            self.color_info = CameraInfo()
            self.color_info.height = color_intr.height
            self.color_info.width = color_intr.width
            self.color_info.distortion_model = "plumb_bob"
            self.color_info.d = list(color_intr.coeffs)
            self.color_info.k = [
                color_intr.fx, 0.0, color_intr.ppx,
                0.0, color_intr.fy, color_intr.ppy,
                0.0, 0.0, 1.0
            ]
            self.color_info.p = [
                color_intr.fx, 0.0, color_intr.ppx, 0.0,
                0.0, color_intr.fy, color_intr.ppy, 0.0,
                0.0, 0.0, 1.0, 0.0
            ]
            self.color_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            
            self.get_logger().info(f'相机内参获取成功: {color_intr.width}x{color_intr.height}')
            
        except Exception as e:
            self.get_logger().warning(f'获取相机内参失败: {str(e)}')

    def publish_data(self):
        """发布数据"""
        try:
            # 等待一组帧
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
            
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            
            if not color_frame or not depth_frame:
                self.get_logger().warning('未获取到完整的帧数据')
                return
            
            # 使用统一的时间戳
            timestamp = self.get_clock().now().to_msg()
            
            # 发布所有TF变换
            self.publish_all_tf(timestamp)
            
            # 设置相机信息
            self.color_info.header.stamp = timestamp
            self.color_info.header.frame_id = "camera_link"  # 改为camera_link以匹配RTAB-Map
            
            # 发布彩色图像
            color_image = np.asanyarray(color_frame.get_data())
            color_msg = self.bridge.cv2_to_imgmsg(color_image, "bgr8")
            color_msg.header.stamp = timestamp
            color_msg.header.frame_id = "camera_link"  # 改为camera_link以匹配RTAB-Map
            self.color_pub.publish(color_msg)
            
            # 发布深度图像
            depth_image = np.asanyarray(depth_frame.get_data())
            depth_msg = self.bridge.cv2_to_imgmsg(depth_image, "16UC1")  # 明确指定编码
            depth_msg.header.stamp = timestamp
            depth_msg.header.frame_id = "camera_link"  # 改为camera_link以匹配RTAB-Map
            self.depth_pub.publish(depth_msg)
            
            # 发布相机信息
            self.color_info_pub.publish(self.color_info)

            # 发布 ascamera 兼容 CameraInfo（不复用引用）
            asc_color_info = CameraInfo()
            asc_color_info.height = self.color_info.height
            asc_color_info.width = self.color_info.width
            asc_color_info.distortion_model = self.color_info.distortion_model
            asc_color_info.d = list(self.color_info.d)
            asc_color_info.k = list(self.color_info.k)
            asc_color_info.p = list(self.color_info.p)
            asc_color_info.r = list(self.color_info.r)
            asc_color_info.header.stamp = timestamp
            asc_color_info.header.frame_id = 'ascamera_camera_link_0'
            self.asc_color_info_pub.publish(asc_color_info)

            # 发布 ascamera 兼容的图像（深拷贝以避免共享 header 导致问题）
            asc_color_msg = copy.deepcopy(color_msg)
            asc_color_msg.header.frame_id = 'ascamera_camera_link_0'
            self.asc_color_pub.publish(asc_color_msg)

            asc_depth_msg = copy.deepcopy(depth_msg)
            asc_depth_msg.header.frame_id = 'ascamera_camera_link_0'
            self.asc_depth_pub.publish(asc_depth_msg)

            self.get_logger().info('发布RGB-D数据', throttle_duration_sec=5)
            
        except Exception as e:
            self.get_logger().warning(f'发布数据时出错: {str(e)}')

    def publish_all_tf(self, timestamp):
        """发布所有必要的TF变换"""
        # 1. 发布odom -> base_link (固定变换)
        transform_odom = TransformStamped()
        transform_odom.header.stamp = timestamp
        transform_odom.header.frame_id = 'odom'
        transform_odom.child_frame_id = 'base_link'
        transform_odom.transform.translation.x = 0.0
        transform_odom.transform.translation.y = 0.0
        transform_odom.transform.translation.z = 0.0
        transform_odom.transform.rotation.x = 0.0
        transform_odom.transform.rotation.y = 0.0
        transform_odom.transform.rotation.z = 0.0
        transform_odom.transform.rotation.w = 1.0
        
        # 2. 发布base_link -> camera_link (相机安装位置)
        transform_camera = TransformStamped()
        transform_camera.header.stamp = timestamp
        transform_camera.header.frame_id = 'base_link'
        transform_camera.child_frame_id = 'camera_link'
        transform_camera.transform.translation.x = 0.1  # 相机在base_link前方0.1米
        transform_camera.transform.translation.y = 0.0
        transform_camera.transform.translation.z = 0.2  # 相机在base_link上方0.2米
        transform_camera.transform.rotation.x = 0.0
        transform_camera.transform.rotation.y = 0.0
        transform_camera.transform.rotation.z = 0.0
        transform_camera.transform.rotation.w = 1.0
        
        # 3. 发布camera_link -> camera_color_optical_frame (光学坐标系)
        transform_optical = TransformStamped()
        transform_optical.header.stamp = timestamp
        transform_optical.header.frame_id = 'camera_link'
        transform_optical.child_frame_id = 'camera_color_optical_frame'
        # 从相机坐标系到光学坐标系的旋转 (绕Y轴旋转-90度，再绕Z轴旋转-90度)
        transform_optical.transform.translation.x = 0.0
        transform_optical.transform.translation.y = 0.0
        transform_optical.transform.translation.z = 0.0
        transform_optical.transform.rotation.x = 0.5
        transform_optical.transform.rotation.y = -0.5
        transform_optical.transform.rotation.z = -0.5
        transform_optical.transform.rotation.w = 0.5
        
        # 同时发布 ascamera 命名空间下的相机框架，保持向后兼容
        transform_ascamera = TransformStamped()
        transform_ascamera.header.stamp = timestamp
        transform_ascamera.header.frame_id = 'base_link'
        transform_ascamera.child_frame_id = 'ascamera_camera_link_0'
        transform_ascamera.transform = transform_camera.transform

        transform_asc_optical = TransformStamped()
        transform_asc_optical.header.stamp = timestamp
        transform_asc_optical.header.frame_id = 'ascamera_camera_link_0'
        transform_asc_optical.child_frame_id = 'ascamera_camera_color_optical_frame'
        transform_asc_optical.transform = transform_optical.transform

        self.tf_broadcaster.sendTransform([transform_odom, transform_camera, transform_optical, transform_ascamera, transform_asc_optical])

    def destroy_node(self):
        """清理资源"""
        self.get_logger().info('正在关闭相机管道...')
        self.pipeline.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = D43515HzNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"节点运行错误: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()