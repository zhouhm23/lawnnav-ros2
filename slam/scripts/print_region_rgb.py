#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import struct
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

class RegionRGBPrinter(Node):
    def __init__(self):
        super().__init__('region_rgb_printer')
        # parameters
        self.declare_parameter('cloud_topic', '/rtabmap/cloud_map')
        self.declare_parameter('xmin', 0.0)
        self.declare_parameter('xmax', 1.0)
        self.declare_parameter('ymin', -0.5)
        self.declare_parameter('ymax', 0.5)
        self.declare_parameter('zmin', -0.2)
        self.declare_parameter('zmax', 0.2)
        self.declare_parameter('sample_limit', 50)

        self.cloud_topic = self.get_parameter('cloud_topic').get_parameter_value().string_value
        self.xmin = self.get_parameter('xmin').get_parameter_value().double_value
        self.xmax = self.get_parameter('xmax').get_parameter_value().double_value
        self.ymin = self.get_parameter('ymin').get_parameter_value().double_value
        self.ymax = self.get_parameter('ymax').get_parameter_value().double_value
        self.zmin = self.get_parameter('zmin').get_parameter_value().double_value
        self.zmax = self.get_parameter('zmax').get_parameter_value().double_value
        self.sample_limit = int(self.get_parameter('sample_limit').get_parameter_value().integer_value)

        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscription = self.create_subscription(PointCloud2, self.cloud_topic, self.cloud_cb, qos_profile=qos)
        self.get_logger().info(f"Subscribed to {self.cloud_topic} (printing points in x=[{self.xmin},{self.xmax}] y=[{self.ymin},{self.ymax}] z=[{self.zmin},{self.zmax}])")

    def decode_rgb(self, rgb_field):
        # rgb_field can be packed float or int
        try:
            # if float packed into 32-bit
            if isinstance(rgb_field, float):
                v = struct.unpack('I', struct.pack('f', rgb_field))[0]
            else:
                v = int(rgb_field)
        except Exception:
            try:
                v = int(rgb_field)
            except Exception:
                return (0,0,0)
        r = (v >> 16) & 0xFF
        g = (v >> 8) & 0xFF
        b = v & 0xFF
        return (r,g,b)

    def cloud_cb(self, msg: PointCloud2):
        fields = [f.name for f in msg.fields]
        has_rgb = 'rgb' in fields or ('r' in fields and 'g' in fields and 'b' in fields)
        if has_rgb and 'rgb' in fields:
            field_names = ("x","y","z","rgb")
        elif has_rgb:
            field_names = ("x","y","z","r","g","b")
        else:
            field_names = ("x","y","z")

        total=0
        matched=0
        samples=[]
        for p in pc2.read_points(msg, field_names=field_names, skip_nans=True):
            total += 1
            try:
                if has_rgb and 'rgb' in fields:
                    x,y,z,rgb = p
                    r,g,b = self.decode_rgb(rgb)
                elif has_rgb:
                    x,y,z,r,g,b = p
                    r = int(r); g = int(g); b = int(b)
                else:
                    x,y,z = p[0],p[1],p[2]
                    r=g=b=0
            except Exception:
                continue

            if x is None or y is None or z is None:
                continue
            if not (self.xmin <= x <= self.xmax):
                continue
            if not (self.ymin <= y <= self.ymax):
                continue
            if not (self.zmin <= z <= self.zmax):
                continue

            matched += 1
            if len(samples) < self.sample_limit:
                samples.append((x,y,z,r,g,b))

        self.get_logger().info(f"PointCloud received: total_points={total}, in_region={matched}")
        if matched>0:
            self.get_logger().info(f"First {len(samples)} samples (x,y,z,r,g,b):")
            for s in samples:
                self.get_logger().info(f"{s}")


def main(args=None):
    rclpy.init(args=args)
    node = RegionRGBPrinter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
