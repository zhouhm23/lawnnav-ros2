#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from visualization_msgs.msg import MarkerArray

class MarkerLatcher(Node):
    def __init__(self):
        super().__init__('path_coverage_marker_latcher')
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        # Subscriber: try to receive latched/transient messages from path_coverage if available
        self.sub = self.create_subscription(MarkerArray, '/path_coverage_marker', self.cb_marker, qos)

        # Publisher(s): republish with TRANSIENT_LOCAL durability so RViz can keep markers
        pub_qos = QoSProfile(depth=1)
        pub_qos.reliability = ReliabilityPolicy.RELIABLE
        pub_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub_same = self.create_publisher(MarkerArray, '/path_coverage_marker', pub_qos)
        self.pub_latched = self.create_publisher(MarkerArray, '/path_coverage_marker_latched', pub_qos)

        self.get_logger().info('Marker latcher started, subscribing to /path_coverage_marker')

    def cb_marker(self, msg: MarkerArray):
        try:
            # Republish the marker array to both topics
            self.pub_same.publish(msg)
            self.pub_latched.publish(msg)
            self.get_logger().info(f'Republished MarkerArray with {len(msg.markers)} markers')
        except Exception as e:
            self.get_logger().error(f'Failed to republish markers: {e}')


def main(args=None):
    rclpy.init(args=args)
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
