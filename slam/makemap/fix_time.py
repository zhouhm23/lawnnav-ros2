import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import time

class MapTimestampFixer(Node):
    def __init__(self):
        super().__init__('map_timestamp_fixer')
        
        # 使用RViz兼容的QoS设置
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # 发布修复后的地图
        self.pub = self.create_publisher(OccupancyGrid, '/map_fixed', qos_profile)
        
        # 订阅原始地图
        self.sub = self.create_subscription(
            OccupancyGrid, 
            '/map', 
            self.map_callback, 
            10
        )
        
        self.get_logger().info("地图时间戳修复器已启动")
        
    def map_callback(self, msg):
        # 修复时间戳 - 使用当前时间
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # 同时修复地图加载时间
        msg.info.map_load_time = self.get_clock().now().to_msg()
        
        self.pub.publish(msg)
        self.get_logger().info(f"修复时间戳并发布地图: {msg.info.width}x{msg.info.height}")

def main():
    rclpy.init()
    node = MapTimestampFixer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()