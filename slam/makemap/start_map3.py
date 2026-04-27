#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import subprocess
import time
import os
import signal
import sys
import threading
import yaml

class MapPublisher(Node):
    def __init__(self, map_yaml_path):
        super().__init__('map_publisher')
        self.qos_profile = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.map_publisher = self.create_publisher(OccupancyGrid, '/map', self.qos_profile)
        self.map_yaml_path = map_yaml_path
        self.map_data = self.load_map_data()
        if self.map_data:
            self.get_logger().info('Map data loaded successfully. Publishing map...')
            self.publish_map()
            self.timer = self.create_timer(2.0, self.publish_map)
        else:
            self.get_logger().error('Failed to load map data. Map will not be published.')

    def load_map_data(self):
        try:
            with open(self.map_yaml_path, 'r') as f:
                map_info = yaml.safe_load(f)
            map_dir = os.path.dirname(self.map_yaml_path)
            pgm_path = os.path.join(map_dir, map_info['image'])
            if not os.path.exists(pgm_path):
                self.get_logger().error(f"PGM file not found at {pgm_path}")
                return None
            resolution = map_info.get('resolution', 0.05)
            origin = map_info.get('origin', [0.0, 0.0, 0.0])
            negate = map_info.get('negate', 0)
            occupied_thresh = map_info.get('occupied_thresh', 0.65)
            free_thresh = map_info.get('free_thresh', 0.196)
            with open(pgm_path, 'rb') as f:
                header = f.readline().decode().strip()
                if header != 'P5': return None
                dimensions = f.readline().decode().strip()
                while dimensions.startswith('#'):
                    dimensions = f.readline().decode().strip()
                width, height = map(int, dimensions.split())
                f.readline() # max_val
                image_data = f.read()
            occupancy_data = []
            for byte in image_data:
                pixel = int(byte)
                if negate: pixel = 255 - pixel
                prob = (255 - pixel) / 255.0
                if prob > occupied_thresh: occupancy_data.append(100)
                elif prob < free_thresh: occupancy_data.append(0)
                else: occupancy_data.append(-1)
            return {'width': width, 'height': height, 'resolution': resolution, 'origin': origin, 'data': occupancy_data}
        except Exception as e:
            self.get_logger().error(f"Failed to load map data: {e}")
            return None

    def publish_map(self):
        if not self.map_data: return
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'map'
        map_msg.info.resolution = self.map_data['resolution']
        map_msg.info.width = self.map_data['width']
        map_msg.info.height = self.map_data['height']
        map_msg.info.origin.position.x = self.map_data['origin'][0]
        map_msg.info.origin.position.y = self.map_data['origin'][1]
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.w = 1.0
        map_msg.data = self.map_data['data']
        map_msg.info.map_load_time = self.get_clock().now().to_msg()
        self.map_publisher.publish(map_msg)

class CoverageLauncher:
    def __init__(self):
        self.processes = []
        self.map_yaml_path = "/home/ubuntu/ros2_ws/src/slam/makemap/maps/final_map.yaml"
        if not os.path.exists(self.map_yaml_path):
            print(f"Map file not found: {self.map_yaml_path}", file=sys.stderr)
            sys.exit(1)
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

    def start_map_publisher(self):
        print("Starting map publisher node...")
        self.map_node = MapPublisher(self.map_yaml_path)
        self.ros_thread = threading.Thread(target=lambda: rclpy.spin(self.map_node))
        self.ros_thread.daemon = True
        self.ros_thread.start()
        time.sleep(2)

    def start_rviz(self):
        print("Starting RViz...")
        rviz_config = """
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views
  - Class: rviz_common/Tool Properties
    Name: Tool Properties
Visualization Manager:
  Global Options:
    Fixed Frame: map
    Frame Rate: 30
  Tools:
    - Class: rviz_default_plugins/Interact
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/PublishPoint
      Topic: /clicked_point
  Displays:
    - Name: Map
      Class: rviz_default_plugins/Map
      Enabled: true
      Topic: /map
      Color Scheme: map
    - Name: Robot Model
      Class: rviz_default_plugins/RobotModel
      Enabled: true
    - Name: TF
      Class: rviz_default_plugins/TF
      Enabled: true
Window Geometry:
  Height: 1000
  Width: 1900
"""
        rviz_config_path = "/tmp/coverage_config.rviz"
        with open(rviz_config_path, 'w') as f:
            f.write(rviz_config)
        rviz_proc = subprocess.Popen(["rviz2", "-d", rviz_config_path])
        self.processes.append(rviz_proc)

    def signal_handler(self, sig, frame):
        print("\nSignal received, cleaning up and shutting down...", file=sys.stderr)
        self.cleanup()
        sys.exit(0)

    def cleanup(self):
        print("Terminating all processes...")
        for proc in reversed(self.processes):
            if proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait(timeout=2)
        if hasattr(self, 'map_node') and self.map_node.context.ok():
            self.map_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if hasattr(self, 'ros_thread') and self.ros_thread.is_alive():
            self.ros_thread.join()
        print("Cleanup complete.")

    def run(self):
        try:
            rclpy.init()
            self.start_map_publisher()
            self.start_rviz()
            print("\nMap display system is running.")
            print("Use the 'Publish Point' tool in RViz to select points.")
            print("Press Ctrl+C in this terminal to shut down.")
            while rclpy.ok():
                time.sleep(1)
        except KeyboardInterrupt:
            pass
        finally:
            self.cleanup()

def main():
    launcher = CoverageLauncher()
    launcher.run()

if __name__ == "__main__":
    main()