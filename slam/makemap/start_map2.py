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
        
        self.map_publisher = self.create_publisher(
            OccupancyGrid, 
            '/map', 
            self.qos_profile
        )
        
        self.map_yaml_path = map_yaml_path
        self.map_data = self.load_map_data()
        
        if self.map_data:
            self.publish_map()
            self.timer = self.create_timer(2.0, self.publish_map)
            
    def load_map_data(self):
        try:
            with open(self.map_yaml_path, 'r') as f:
                map_info = yaml.safe_load(f)
            
            map_dir = os.path.dirname(self.map_yaml_path)
            pgm_path = os.path.join(map_dir, map_info['image'])
            
            if not os.path.exists(pgm_path):
                return None
            
            resolution = map_info.get('resolution', 0.05)
            origin = map_info.get('origin', [0.0, 0.0, 0.0])
            negate = map_info.get('negate', 0)
            occupied_thresh = map_info.get('occupied_thresh', 0.65)
            free_thresh = map_info.get('free_thresh', 0.196)
            
            with open(pgm_path, 'rb') as f:
                header = f.readline().decode().strip()
                if header != 'P5':
                    return None
                
                dimensions = f.readline().decode().strip()
                while dimensions.startswith('#'):
                    dimensions = f.readline().decode().strip()
                
                width, height = map(int, dimensions.split())
                max_val = int(f.readline().decode().strip())
                image_data = f.read()
            
            occupancy_data = []
            for byte in image_data:
                pixel = int(byte)
                
                if negate:
                    pixel = 255 - pixel
                
                if pixel < 255 * free_thresh:
                    occupancy_data.append(0)
                elif pixel > 255 * occupied_thresh:
                    occupancy_data.append(100)
                else:
                    occupancy_data.append(-1)
            
            fixed_data = self.horizontal_flip(occupancy_data, width, height)
            
            return {
                'width': width,
                'height': height,
                'resolution': resolution,
                'origin': origin,
                'data': fixed_data
            }
            
        except Exception as e:
            return None
    
    def horizontal_flip(self, data, width, height):
        flipped_data = []
        for row in range(height):
            start_index = row * width
            end_index = start_index + width
            row_data = data[start_index:end_index]
            flipped_data.extend(row_data[::-1])
        return flipped_data
            
    def publish_map(self):
        if not self.map_data:
            return
            
        map_msg = OccupancyGrid()
        map_msg.header = Header()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'map'
        
        map_msg.info.resolution = self.map_data['resolution']
        map_msg.info.width = self.map_data['width']
        map_msg.info.height = self.map_data['height']
        
        map_msg.info.origin.position.x = self.map_data['origin'][0]
        map_msg.info.origin.position.y = self.map_data['origin'][1]
        map_msg.info.origin.position.z = self.map_data['origin'][2]
        map_msg.info.origin.orientation.x = 0.0
        map_msg.info.origin.orientation.y = 0.0
        map_msg.info.origin.orientation.z = 0.0
        map_msg.info.origin.orientation.w = 1.0
        
        map_msg.data = self.map_data['data']
        map_msg.info.map_load_time = self.get_clock().now().to_msg()
        
        self.map_publisher.publish(map_msg)

class MapLauncher:
    def __init__(self):
        self.processes = []
        self.map_yaml_path = "/home/ubuntu/ros2_ws/src/slam/makemap/maps/final_map.yaml"
        
        if not os.path.exists(self.map_yaml_path):
            print(f"地图文件不存在: {self.map_yaml_path}")
            sys.exit(1)
            
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
    def start_tf(self):
        tf_proc = subprocess.Popen([
            "ros2", "run", "tf2_ros", "static_transform_publisher", 
            "0", "0", "0", "0", "0", "0", "map", "odom"
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.processes.append(tf_proc)
        time.sleep(1)
        
    def start_map_publisher(self):
        rclpy.init()
        self.map_node = MapPublisher(self.map_yaml_path)
        self.ros_thread = threading.Thread(target=self.spin_node)
        self.ros_thread.daemon = True
        self.ros_thread.start()
        time.sleep(2)
        
    def spin_node(self):
        try:
            rclpy.spin(self.map_node)
        except KeyboardInterrupt:
            pass
            
    def start_rviz(self):
        rviz_config = """
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Selection
    Name: Selection
  - Class: rviz_common/Tool Properties
    Name: Tool Properties
  - Class: rviz_common/Views
    Name: Views
Visualization Manager:
  Displays:
    - Class: rviz_default_plugins/Grid
      Enabled: true
      Name: Grid
      Reference Frame: map
      Cell Size: 0.5
      Plane: XY
    - Class: rviz_default_plugins/Map
      Enabled: true
      Name: Map
      Topic: /map
      Color Scheme: costmap
      Draw Behind: true
      Use Timestamp: false
    - Class: rviz_default_plugins/TF
      Enabled: false
      Name: TF
  Global Options:
    Fixed Frame: map
    Background Color: 48; 48; 48
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 10.0
      Name: Current View
      Target Frame: map
      Focal Point:
        X: 0.0
        Y: 0.0
        Z: 0.0
      Pitch: 0.5
      Yaw: 0.0
  Tools:
    - Class: rviz_default_plugins/Interact
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
Window Geometry:
  Height: 800
  Width: 1200
  X: 100
  Y: 100
"""
        rviz_config_path = "/tmp/map_config.rviz"
        with open(rviz_config_path, 'w') as f:
            f.write(rviz_config)
            
        rviz_proc = subprocess.Popen([
            "rviz2", "-d", rviz_config_path
        ])
        self.processes.append(rviz_proc)
    
    def signal_handler(self, sig, frame):
        self.cleanup()
        sys.exit(0)
        
    def cleanup(self):
        if hasattr(self, 'map_node'):
            self.map_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
            
        for proc in self.processes:
            if proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    
    def run(self):
        try:
            self.start_tf()
            self.start_map_publisher()
            self.start_rviz()
            
            print("地图系统已启动")
            print("按 Ctrl+C 退出")
            
            while True:
                time.sleep(1)
                
        except Exception as e:
            self.cleanup()

def main():
    launcher = MapLauncher()
    launcher.run()

if __name__ == "__main__":
    main()