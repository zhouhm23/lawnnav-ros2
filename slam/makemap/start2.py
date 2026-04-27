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

class D43515HzNode(Node):
    def __init__(self):
        super().__init__('d435_15hz_node')
        
        self.get_logger().info('正在初始化D435节点...')
        
        # 创建发布者
        self.color_pub = self.create_publisher(Image, '/rgb/image', 10)
        self.depth_pub = self.create_publisher(Image, '/depth/image', 10)
        self.color_info_pub = self.create_publisher(CameraInfo, '/rgb/camera_info', 10)
        
        # 创建TF广播器
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        
        self.bridge = CvBridge()
        
        # 发布静态TF变换（base_link -> camera_color_optical_frame）
        self.publish_static_tf()
        
        # 初始化相机（降低分辨率）
        self.init_camera_low_res()
        
        # 创建定时器 - 15Hz
        self.timer = self.create_timer(0.067, self.publish_data)  # ~15Hz
        
        self.get_logger().info('D435节点初始化完成，分辨率：424x240')

    def publish_static_tf(self):
        """发布静态TF变换 - 避免时间同步问题"""
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = 'base_link'
        static_transform.child_frame_id = 'camera_color_optical_frame'
        
        # 假设相机在base_link前方0.1米，高度0.2米
        static_transform.transform.translation.x = 0.1
        static_transform.transform.translation.y = 0.0
        static_transform.transform.translation.z = 0.2
        static_transform.transform.rotation.x = 0.0
        static_transform.transform.rotation.y = 0.0
        static_transform.transform.rotation.z = 0.0
        static_transform.transform.rotation.w = 1.0
        
        self.static_tf_broadcaster.sendTransform(static_transform)
        self.get_logger().info('已发布静态TF: base_link -> camera_color_optical_frame')

    def init_camera_low_res(self):
        """初始化相机 - 降低分辨率以提高性能"""
        try:
            self.pipeline = rs.pipeline()
            config = rs.config()
            
            # 降低分辨率配置
            # 选项1: 424x240 (最低分辨率，最高性能)
            # 选项2: 480x270 (平衡选项)
            # 选项3: 640x360 (中等分辨率)
            
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
            
            # 发布动态TF（odom -> base_link）
            self.publish_dynamic_tf(timestamp)
            
            # 设置相机信息
            self.color_info.header.stamp = timestamp
            self.color_info.header.frame_id = "camera_color_optical_frame"
            
            # 发布彩色图像
            color_image = np.asanyarray(color_frame.get_data())
            color_msg = self.bridge.cv2_to_imgmsg(color_image, "bgr8")
            color_msg.header.stamp = timestamp
            color_msg.header.frame_id = "camera_color_optical_frame"
            self.color_pub.publish(color_msg)
            
            # 发布深度图像
            depth_image = np.asanyarray(depth_frame.get_data())
            depth_msg = self.bridge.cv2_to_imgmsg(depth_image, "passthrough")
            depth_msg.header.stamp = timestamp
            depth_msg.header.frame_id = "camera_color_optical_frame"
            self.depth_pub.publish(depth_msg)
            
            # 发布相机信息
            self.color_info_pub.publish(self.color_info)
            
            self.get_logger().info('发布RGB-D数据', throttle_duration_sec=5)
            
        except Exception as e:
            self.get_logger().warning(f'发布数据时出错: {str(e)}')

    def publish_dynamic_tf(self, timestamp):
        """发布动态TF变换（odom -> base_link）"""
        transform = TransformStamped()
        transform.header.stamp = timestamp
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(transform)

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