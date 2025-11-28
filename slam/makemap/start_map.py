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

class FixedMapPublisher(Node):
    def __init__(self, map_yaml_path):
        super().__init__('fixed_map_publisher')
        
        # 使用RViz兼容的QoS设置
        self.qos_profile = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # 创建地图发布者
        self.map_publisher = self.create_publisher(
            OccupancyGrid, 
            '/map', 
            self.qos_profile
        )
        
        # 加载地图数据
        self.map_yaml_path = map_yaml_path
        self.map_data = self.load_map_data()
        
        if self.map_data:
            self.get_logger().info(f"地图加载成功: {self.map_data['width']}x{self.map_data['height']}")
            # 立即发布地图
            self.publish_map()
            # 定时发布以确保RViz收到
            self.timer = self.create_timer(2.0, self.publish_map)
        else:
            self.get_logger().error("地图加载失败")
            
    def load_map_data(self):
        """从YAML文件加载地图数据并读取PGM文件，修复翻转问题"""
        try:
            with open(self.map_yaml_path, 'r') as f:
                map_info = yaml.safe_load(f)
            
            # 读取PGM文件路径
            map_dir = os.path.dirname(self.map_yaml_path)
            pgm_path = os.path.join(map_dir, map_info['image'])
            
            if not os.path.exists(pgm_path):
                self.get_logger().error(f"PGM文件不存在: {pgm_path}")
                return None
            
            # 获取地图参数
            resolution = map_info.get('resolution', 0.05)
            origin = map_info.get('origin', [0.0, 0.0, 0.0])
            negate = map_info.get('negate', 0)
            occupied_thresh = map_info.get('occupied_thresh', 0.65)
            free_thresh = map_info.get('free_thresh', 0.196)
            
            # 读取PGM文件
            with open(pgm_path, 'rb') as f:
                # 读取PGM头信息
                header = f.readline().decode().strip()
                if header != 'P5':
                    self.get_logger().error(f"不支持的PGM格式: {header}")
                    return None
                
                # 读取尺寸
                dimensions = f.readline().decode().strip()
                while dimensions.startswith('#'):
                    dimensions = f.readline().decode().strip()
                
                width, height = map(int, dimensions.split())
                
                # 读取最大值
                max_val = int(f.readline().decode().strip())
                
                # 读取图像数据
                image_data = f.read()
            
            # 将图像数据转换为占用网格数据
            occupancy_data = []
            for byte in image_data:
                pixel = int(byte)
                
                # 根据negate参数处理
                if negate:
                    pixel = 255 - pixel
                
                # 转换为占用值 (0-100)
                if pixel < 255 * free_thresh:
                    occupancy_data.append(0)  # 空闲
                elif pixel > 255 * occupied_thresh:
                    occupancy_data.append(100)  # 占据
                else:
                    occupancy_data.append(-1)  # 未知
            
            # 修复翻转问题 - 水平翻转数据
            # 这是因为ROS地图坐标系与图像坐标系方向可能不同
            fixed_data = self.horizontal_flip(occupancy_data, width, height)
            
            return {
                'width': width,
                'height': height,
                'resolution': resolution,
                'origin': origin,
                'data': fixed_data
            }
            
        except Exception as e:
            self.get_logger().error(f"加载地图数据失败: {e}")
            return None
    
    def horizontal_flip(self, data, width, height):
        """水平翻转地图数据"""
        flipped_data = []
        for row in range(height):
            start_index = row * width
            end_index = start_index + width
            # 获取当前行并反转
            row_data = data[start_index:end_index]
            flipped_data.extend(row_data[::-1])  # 反转行数据
        return flipped_data
    
    def vertical_flip(self, data, width, height):
        """垂直翻转地图数据"""
        flipped_data = []
        for row in range(height-1, -1, -1):  # 从最后一行开始
            start_index = row * width
            end_index = start_index + width
            flipped_data.extend(data[start_index:end_index])
        return flipped_data
            
    def publish_map(self):
        """发布地图数据"""
        if not self.map_data:
            return
            
        map_msg = OccupancyGrid()
        
        # 设置header
        map_msg.header = Header()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'map'
        
        # 设置地图信息
        map_msg.info.resolution = self.map_data['resolution']
        map_msg.info.width = self.map_data['width']
        map_msg.info.height = self.map_data['height']
        
        # 设置原点
        map_msg.info.origin.position.x = self.map_data['origin'][0]
        map_msg.info.origin.position.y = self.map_data['origin'][1]
        map_msg.info.origin.position.z = self.map_data['origin'][2]
        map_msg.info.origin.orientation.x = 0.0
        map_msg.info.origin.orientation.y = 0.0
        map_msg.info.origin.orientation.z = 0.0
        map_msg.info.origin.orientation.w = 1.0
        
        # 设置地图数据
        map_msg.data = self.map_data['data']
        
        # 发布时间戳
        map_msg.info.map_load_time = self.get_clock().now().to_msg()
        
        self.map_publisher.publish(map_msg)
        self.get_logger().info(f"发布修复后的地图: {map_msg.info.width}x{map_msg.info.height}", 
                              throttle_duration_sec=10)

class FinalMapLauncher:
    def __init__(self):
        self.processes = []
        self.map_yaml_path = "/home/ubuntu/ros2_ws/src/slam/makemap/maps/final_map.yaml"
        
        # 检查地图文件
        if not os.path.exists(self.map_yaml_path):
            print(f"错误: 地图文件不存在: {self.map_yaml_path}")
            sys.exit(1)
            
        # 设置信号处理
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
    def start_tf(self):
        """启动静态TF变换"""
        print("启动静态TF变换...")
        tf_proc = subprocess.Popen([
            "ros2", "run", "tf2_ros", "static_transform_publisher", 
            "0", "0", "0", "0", "0", "0", "map", "odom"
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.processes.append(tf_proc)
        time.sleep(1)
        
    def start_map_publisher(self):
        """启动修复后的地图发布器"""
        print("启动修复后的地图发布器...")
        
        # 初始化ROS2
        rclpy.init()
        
        # 创建地图发布节点
        self.map_node = FixedMapPublisher(self.map_yaml_path)
        
        # 在单独的线程中spin节点
        self.ros_thread = threading.Thread(target=self.spin_node)
        self.ros_thread.daemon = True
        self.ros_thread.start()
        
        # 等待节点启动
        time.sleep(2)
        
    def spin_node(self):
        """在单独线程中spin ROS节点"""
        try:
            rclpy.spin(self.map_node)
        except KeyboardInterrupt:
            pass
            
    def start_rviz_final(self):
        """启动RViz2"""
        print("启动RViz2...")
        
        # 创建RViz配置
        rviz_config = self.create_final_rviz_config()
        rviz_config_path = "/tmp/final_map_config.rviz"
        
        with open(rviz_config_path, 'w') as f:
            f.write(rviz_config)
            
        rviz_proc = subprocess.Popen([
            "rviz2", "-d", rviz_config_path
        ])
        self.processes.append(rviz_proc)
        print("RViz2 已启动")
        
    def create_final_rviz_config(self):
        """创建最终RViz配置"""
        return """Panels:
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
      Color: 128; 128; 128
      Alpha: 0.3
    - Class: rviz_default_plugins/Map
      Enabled: true
      Name: Map
      Topic: /map
      Color Scheme: costmap
      Draw Behind: true
      Use Timestamp: false
      Alpha: 1.0
    - Class: rviz_default_plugins/TF
      Enabled: false
      Name: TF
  Global Options:
    Fixed Frame: map
    Background Color: 48; 48; 48
    Frame Rate: 60
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
    - Class: rviz_default_plugins/SetInitialPose
    - Class: rviz_default_plugins/SetGoal
    - Class: rviz_default_plugins/PublishPoint
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 10.0
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.05999999865889549
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 0.0
        Y: 0.0
        Z: 0.0
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05000000074505806
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.009999999776482582
      Pitch: 0.785398006439209
      Target Frame: map
      Value: Orbit (rviz)
      Yaw: 0.785398006439209
    Saved: ~
Window Geometry:
  Displays:
    collapsed: false
  Height: 800
  Hide Left Dock: false
  Hide Right Dock: false
  QMainWindow State: 000000ff00000000fd000000040000000000000156000002f4fc0200000008fb0000001200530065006c0065006300740069006f006e00000001e10000009b0000005c00fffffffb0000001e0054006f006f006c002000500072006f007000650072007400690065007302000001ed000001df00000185000000a3fb000000120056006900650077007300200054006f006f02000001df000002110000018500000122fb000000200054006f006f006c002000500072006f0070006500720074006900650073003203000002880000011d000002210000017afb000000100044006900730070006c006100790073010000003d000002f4000000c900fffffffb0000002000730065006c0065006300740069006f006e00200062007500660066006500720200000138000000aa0000023a00000294fb00000014005700690064006500670065007400730100000041000000e60000000000000000fb0000000c004b0069006e0065006300740200000186000001060000030c00000261000000010000010f000002f4fc0200000003fb0000001e0054006f006f006c002000500072006f00700065007200740069006500730100000041000000780000000000000000fb0000000a00560069006500770073010000003d000002f4000000a400fffffffb0000001200530065006c0065006300740069006f006e010000025a000000b200000000000000000000000200000490000000a9fc0100000001fb0000000a00560069006500770073030000004e00000080000002e10000019700000003000004420000003efc0100000002fb0000000800540069006d00650100000000000004420000000000000000fb0000000800540069006d006501000000000000045000000000000000000000023f000002f400000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000
  Selection:
    collapsed: false
  Tool Properties:
    collapsed: false
  Views:
    collapsed: false
  Width: 1200
  X: 100
  Y: 100
"""
    
    def signal_handler(self, sig, frame):
        """处理中断信号，清理所有进程"""
        print("\n正在关闭所有进程...")
        self.cleanup()
        sys.exit(0)
        
    def cleanup(self):
        """清理所有进程"""
        # 关闭ROS2节点
        if hasattr(self, 'map_node'):
            self.map_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
            
        # 关闭所有子进程
        for proc in self.processes:
            if proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    
    def run(self):
        """主运行函数"""
        print("=" * 60)
        print("最终地图解决方案 - 修复翻转问题")
        print("=" * 60)
        
        try:
            # 1. 启动TF变换
            self.start_tf()
            time.sleep(2)
            
            # 2. 启动修复后的地图发布器
            self.start_map_publisher()
            time.sleep(3)
            
            # 3. 启动RViz
            self.start_rviz_final()
            
            print("\n" + "=" * 60)
            print("所有组件已启动!")
            print("- TF变换: 运行中")
            print("- 修复后的地图发布器: 运行中") 
            print("- RViz2: 已启动")
            print("=" * 60)
            print("地图翻转问题应该已经修复!")
            print("如果地图仍然不正确，请尝试:")
            print("1. 修改脚本中的翻转方向")
            print("2. 检查PGM文件本身的朝向")
            print("按 Ctrl+C 退出")
            
            # 保持主线程运行
            while True:
                time.sleep(1)
                
        except Exception as e:
            print(f"启动过程中出现错误: {e}")
            self.cleanup()

def main():
    launcher = FinalMapLauncher()
    launcher.run()

if __name__ == "__main__":
    main()