#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class MarkerLatcher(Node):
    def __init__(self):
        super().__init__('marker_latcher')
        # Publisher uses TRANSIENT_LOCAL durability so last message is kept after publisher dies
        pub_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(MarkerArray, '/path_coverage_marker', pub_qos)
        # Subscriber listens for incoming MarkerArray messages (volatile by default)
        sub_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.sub = self.create_subscription(MarkerArray, '/path_coverage_marker', self.on_marker, sub_qos)
        self.get_logger().info('MarkerLatcher started: republishing /path_coverage_marker with TRANSIENT_LOCAL')

    def on_marker(self, msg: MarkerArray):
        try:
            # Republish received message so the transient_local publisher latches it
            self.pub.publish(msg)
            self.get_logger().info(f'Republished MarkerArray with {len(msg.markers)} markers')
        except Exception as e:
            self.get_logger().error(f'Failed to republish marker array: {e}')

def main():
    rclpy.init()
    node = MarkerLatcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
