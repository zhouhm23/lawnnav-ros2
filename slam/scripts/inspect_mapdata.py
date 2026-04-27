#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# Try to import MapData message type; if unavailable, user can run `ros2 topic echo /mapData --once`
try:
    from rtabmap_msgs.msg import MapData
    HAS_TYPE = True
except Exception:
    MapData = None
    HAS_TYPE = False

import sys

class MapDataInspector(Node):
    def __init__(self):
        super().__init__('mapdata_inspector')
        self.declare_parameter('topic', '/mapData')
        self.topic = self.get_parameter('topic').get_parameter_value().string_value
        if HAS_TYPE:
            self.get_logger().info(f'Subscribing to {self.topic} as rtabmap_msgs/MapData')
            self.sub = self.create_subscription(MapData, self.topic, self.cb, 10)
        else:
            self.get_logger().warning('rtabmap_msgs not available in this env; please run `ros2 topic echo /mapData --once` instead')
            sys.exit(1)

    def cb(self, msg):
        # recursively print structure
        def repr_field(value, depth=0):
            indent = '  ' * depth
            if isinstance(value, list):
                print(f"{indent}list[len={len(value)}]")
                if len(value) > 0:
                    sample = value[0]
                    repr_field(sample, depth+1)
            elif hasattr(value, '__slots__'):
                print(f"{indent}{value.__class__.__name__}:")
                for s in value.__slots__:
                    try:
                        v = getattr(value, s)
                    except Exception:
                        v = '<error>'
                    print(f"{indent}  {s}: ")
                    repr_field(v, depth+2)
            else:
                # primitive or numpy
                try:
                    print(f"{indent}{repr(value)}")
                except Exception:
                    print(f"{indent}<unprintable>")

        print('--- MapData message structure ---')
        repr_field(msg)
        print('--- End ---')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = MapDataInspector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
