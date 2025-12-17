#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
import numpy as np

class MapColorOverlay(Node):
    def __init__(self):
        super().__init__('map_color_overlay')
        
        # Parameters
        self.declare_parameter('cloud_topic', '/rtabmap/cloud_map')
        self.declare_parameter('marker_topic', '/map_color_markers')
        self.declare_parameter('min_green', 25)
        self.declare_parameter('green_ratio', 1.05)
        self.declare_parameter('min_z', -0.1) # Ground level approx
        self.declare_parameter('max_z', 0.1)
        self.declare_parameter('marker_size', 0.05)

        self.cloud_topic = self.get_parameter('cloud_topic').get_parameter_value().string_value
        self.marker_topic = self.get_parameter('marker_topic').get_parameter_value().string_value
        self.min_green = self.get_parameter('min_green').get_parameter_value().integer_value
        self.green_ratio = self.get_parameter('green_ratio').get_parameter_value().double_value
        self.min_z = self.get_parameter('min_z').get_parameter_value().double_value
        self.max_z = self.get_parameter('max_z').get_parameter_value().double_value
        self.marker_size = self.get_parameter('marker_size').get_parameter_value().double_value

        # QoS Profile for cloud subscription (TRANSIENT_LOCAL to match rtabmap)
        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.cloud_sub = self.create_subscription(
            PointCloud2, 
            self.cloud_topic, 
            self.cloud_cb, 
            qos_profile=qos_profile)
            
        self.marker_pub = self.create_publisher(Marker, self.marker_topic, 10)
        
        self.get_logger().info(f'Subscribed to {self.cloud_topic}, publishing markers to {self.marker_topic}')

    def cloud_cb(self, cloud: PointCloud2):
        self.get_logger().info(f'Received cloud with {cloud.width * cloud.height} points')
        
        # Check fields
        fields = [f.name for f in cloud.fields]
        has_rgb = 'rgb' in fields
        if not has_rgb:
            return

        # Read points
        gen = pc2.read_points(cloud, field_names=("x", "y", "z", "rgb"), skip_nans=True)
        
        green_points = []
        
        for p in gen:
            x, y, z, rgb_float = p
            
            # Height filter
            if not (self.min_z <= z <= self.max_z):
                continue

            # Decode RGB
            try:
                rgb_int = int(np.float32(rgb_float).view(np.uint32))
            except:
                continue
                
            r = (rgb_int >> 16) & 0xFF
            g = (rgb_int >> 8) & 0xFF
            b = (rgb_int) & 0xFF
            
            # Green detection
            is_green = (g > self.min_green) and (g > r * self.green_ratio) and (g > b * self.green_ratio)
            
            # Special case for dark green mats
            if not is_green and g > 8 and g > r + 4 and g > b + 4:
                 is_green = True
            
            if is_green:
                pt = Point()
                pt.x = float(x)
                pt.y = float(y)
                pt.z = float(z)
                green_points.append(pt)
                
        if not green_points:
            self.get_logger().info('No green points found.')
            return

        # Quantize green points into grid cells (centered) and deduplicate so we render
        # contiguous flat tiles instead of many tiny cubes.
        thickness = max(self.marker_size * 0.02, 0.001)
        cell_set = set()
        cells = []
        for pt in green_points:
            gx = round(pt.x / self.marker_size) * self.marker_size
            gy = round(pt.y / self.marker_size) * self.marker_size
            gz = max(self.min_z, min(self.max_z, pt.z)) + thickness * 0.5
            key = (int(round(gx * 1000)), int(round(gy * 1000)))
            if key in cell_set:
                continue
            cell_set.add(key)
            p = Point()
            p.x = float(gx)
            p.y = float(gy)
            p.z = float(gz)
            cells.append(p)

        if not cells:
            self.get_logger().info('No grid cells to publish after quantization.')
            return

        marker = Marker()
        marker.header.frame_id = cloud.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "green_overlay"
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = float(self.marker_size)
        marker.scale.y = float(self.marker_size)
        marker.scale.z = float(thickness)
        # pale/light green
        marker.color.r = 0.6
        marker.color.g = 1.0
        marker.color.b = 0.6
        marker.color.a = 0.6
        marker.points = cells

        self.marker_pub.publish(marker)
        self.get_logger().info(f'Published {len(cells)} grid tiles (from {len(green_points)} points).')



def main(args=None):
    rclpy.init(args=args)
    node = MapColorOverlay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()