#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import os
import subprocess
import time
import signal
import sys
import yaml
from PIL import Image
import cv2

class AutoSLAM(Node):
    def __init__(self):
        super().__init__('auto_slam')
        
        # 获取当前脚本所在目录
        self.script_dir = os.path.dirname(os.path.abspath(__file__))
        self.output_dir = os.path.join(self.script_dir, 'maps')
        os.makedirs(self.output_dir, exist_ok=True)
        
        # 参数 - 使用您提供的时间设置
        self.total_duration = 70  # 总运行时间40秒
        self.save_trigger_time = 60  # 在第30秒触发保存
        self.cell_size = 0.05  # 5cm栅格大小
        self.height_min = -0.2  # 最小高度-20cm
        self.height_max = 0.0   # 最大高度0cm
        
        # 进程和状态
        self.rtabmap_process = None
        self.camera_process = None
        self.map_saved = False
        self.start_time = time.time()
        self.shutting_down = False
        
        # 点云数据
        self.pointcloud_data = []  # 存储所有点云数据
        self.pointcloud_received = False
        
        # 订阅点云话题
        self.subscription = self.create_subscription(
            PointCloud2,
            '/rtabmap/cloud_map',
            self.pointcloud_callback,
            10)
        
        self.get_logger().info('Auto SLAM system initializing...')
        self.get_logger().info(f'Will filter points at height: {self.height_min}-{self.height_max}m')
        
        # 启动系统
        self.start_system()
        
        # 设置定时器
        self.timer = self.create_timer(1.0, self.check_timer)
        
        self.get_logger().info(f'System will run for {self.total_duration} seconds, saving at {self.save_trigger_time} seconds')

    def start_system(self):
        """启动相机和RTAB-Map"""
        try:
            # 启动相机节点
            camera_script = os.path.join(self.script_dir, 'start3.py')
            if os.path.exists(camera_script):
                self.get_logger().info('Starting camera node...')
                self.camera_process = subprocess.Popen(
                    [sys.executable, camera_script],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    preexec_fn=os.setsid
                )
                time.sleep(3)  # 等待相机启动
            
            # 启动RTAB-Map
            self.get_logger().info('Starting RTAB-Map...')
            
            rtabmap_cmd = [
                'ros2', 'launch', 'rtabmap_launch', 'rtabmap.launch.py',
                'rtabmap_args:=--delete_db_on_start',
                'depth_topic:=/depth/image',
                'rgb_topic:=/rgb/image',
                'camera_info_topic:=/rgb/camera_info',
                'frame_id:=base_link',
                'approx_sync:=true',
                'wait_imu_to_init:=false',
                'odom_frame_id:=odom'
            ]
            
            self.rtabmap_process = subprocess.Popen(
                rtabmap_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid
            )
            
            self.get_logger().info('RTAB-Map started successfully')
            
        except Exception as e:
            self.get_logger().error(f'Failed to start system: {e}')
            self.cleanup_and_exit()

    def check_timer(self):
        """检查是否到达保存或停止时间"""
        if self.shutting_down:
            return
            
        elapsed = time.time() - self.start_time
        
        # 每5秒打印一次状态
        if int(elapsed) % 5 == 0:
            if self.pointcloud_received:
                status = f"receiving pointcloud data ({len(self.pointcloud_data)} points)"
            else:
                status = "waiting for pointcloud data"
            self.get_logger().info(f'Elapsed: {elapsed:.1f}s, Status: {status}')
        
        # 触发保存 - 只在30秒时触发一次
        if elapsed >= self.save_trigger_time and not self.map_saved:
            self.get_logger().info(f'Reached {self.save_trigger_time} seconds, triggering map save...')
            self.trigger_map_save()
            self.map_saved = True  # 标记为已保存，防止重复触发
        
        # 停止系统
        if elapsed >= self.total_duration:
            self.get_logger().info(f'Reached {self.total_duration} seconds, stopping system...')
            self.cleanup_and_exit()

    def pointcloud_callback(self, msg):
        """点云回调函数"""
        if self.map_saved or self.shutting_down:
            return
            
        try:
            if not self.pointcloud_received:
                self.get_logger().info('First pointcloud received!')
                self.get_logger().info(f'PointCloud frame_id: {msg.header.frame_id}')
                
                # 采样前几个点分析坐标轴
                points_gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
                sample_points = []
                for i, point in enumerate(points_gen):
                    if i >= 5:  # 采样5个点
                        break
                    if len(point) == 3:
                        sample_points.append(point)
                        self.get_logger().info(f"Sample point {i}: x={point[0]:.3f}, y={point[1]:.3f}, z={point[2]:.3f}")
                
                # 分析坐标范围
                if sample_points:
                    x_vals = [p[0] for p in sample_points]
                    y_vals = [p[1] for p in sample_points]
                    z_vals = [p[2] for p in sample_points]
                    self.get_logger().info(f"Coordinate ranges - X: {min(x_vals):.3f} to {max(x_vals):.3f}")
                    self.get_logger().info(f"Coordinate ranges - Y: {min(y_vals):.3f} to {max(y_vals):.3f}")
                    self.get_logger().info(f"Coordinate ranges - Z: {min(z_vals):.3f} to {max(z_vals):.3f}")
                
                self.pointcloud_received = True
            
            # 使用更可靠的方法提取点云数据
            points_gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            
            # 逐个处理点，确保每个点都有3个坐标
            new_points = []
            for point in points_gen:
                if len(point) == 3:  # 确保每个点有x,y,z三个坐标
                    new_points.append(point)
            
            # 添加到点云数据列表
            self.pointcloud_data.extend(new_points)
            
            if len(self.pointcloud_data) % 1000 == 0:  # 每1000个点打印一次
                self.get_logger().info(f'Collected {len(self.pointcloud_data)} points so far')
            
        except Exception as e:
            self.get_logger().error(f'Error processing pointcloud: {e}')

    def filter_points_by_height(self):
        """筛选高度在-20cm到0cm的点 - 使用Y轴作为高度"""
        if not self.pointcloud_data:
            self.get_logger().warning('No pointcloud data to filter')
            return []
        
        try:
            self.get_logger().info(f'Total points before filtering: {len(self.pointcloud_data)}')
            
            filtered_points = []
            for point in self.pointcloud_data:
                if len(point) == 3:
                    x, y, z = point
                    # 使用Y坐标作为高度进行过滤，范围-0.2到0
                    if self.height_min <= y <= self.height_max:
                        filtered_points.append((x, y, z))
            
            self.get_logger().info(f'Points after height filtering (Y-axis): {len(filtered_points)}')
            
            return filtered_points
            
        except Exception as e:
            self.get_logger().error(f'Error filtering points by height: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            return []

    def generate_grid_map(self, points):
        """从筛选后的点云生成栅格地图，包含所有象限的点，相机在偏左下位置"""
        if len(points) == 0:
            self.get_logger().warning('No points to generate grid map')
            return None, 0.0, 0.0
        
        try:
            # 提取x和z坐标
            x_coords = [p[0] for p in points]  # 左右方向 (X轴)
            z_coords = [p[2] for p in points]  # 前后方向 (Z轴)
            
            # 计算点云边界
            min_x, max_x = min(x_coords), max(x_coords)
            min_z, max_z = min(z_coords), max(z_coords)
            
            # 确保相机位置(0,0)在地图的偏左下位置
            # 我们将地图边界扩展到相机位置的左侧和下方
            map_margin_x = 2.0  # X方向2米边距
            map_margin_z = 2.0  # Z方向2米边距
            adjusted_min_x = min(min_x, -map_margin_x)
            adjusted_min_z = min(min_z, -map_margin_z)
            adjusted_max_x = max(max_x, map_margin_x)
            adjusted_max_z = max(max_z, map_margin_z)
            
            self.get_logger().info(f'Point cloud range - X: {min_x:.2f} to {max_x:.2f}, Z: {min_z:.2f} to {max_z:.2f}')
            self.get_logger().info(f'Adjusted map range - X: {adjusted_min_x:.2f} to {adjusted_max_x:.2f}, Z: {adjusted_min_z:.2f} to {adjusted_max_z:.2f}')
            
            # 计算地图尺寸
            map_width = max(1, int((adjusted_max_x - adjusted_min_x) / self.cell_size) + 1)
            map_height = max(1, int((adjusted_max_z - adjusted_min_z) / self.cell_size) + 1)
            
            self.get_logger().info(f'Grid map dimensions: {map_width}x{map_height}')
            self.get_logger().info(f'JSON数组尺寸: {map_height}行 x {map_width}列')
            
            # 创建地图
            occupancy_grid = np.zeros((map_height, map_width), dtype=np.uint8)
            
            # 将点云投影到栅格地图
            # 注意：我们需要反转Z方向，确保从下往上是正方向
            occupied_count = 0
            for x, y, z in points:
                grid_x = int((x - adjusted_min_x) / self.cell_size)
                # 反转Z方向：从下往上为正方向
                grid_z = map_height - 1 - int((z - adjusted_min_z) / self.cell_size)
                
                if 0 <= grid_x < map_width and 0 <= grid_z < map_height:
                    occupancy_grid[grid_z, grid_x] = 1
                    occupied_count += 1
            
            # 计算相机在地图中的位置（反转Z方向）
            camera_grid_x = int((0 - adjusted_min_x) / self.cell_size)
            camera_grid_z = map_height - 1 - int((0 - adjusted_min_z) / self.cell_size)
            
            # 计算各象限的点数
            quadrant_counts = {
                "Q1 (X+, Z+)": sum(1 for p in points if p[0] >= 0 and p[2] >= 0),
                "Q2 (X-, Z+)": sum(1 for p in points if p[0] < 0 and p[2] >= 0),
                "Q3 (X-, Z-)": sum(1 for p in points if p[0] < 0 and p[2] < 0),
                "Q4 (X+, Z-)": sum(1 for p in points if p[0] >= 0 and p[2] < 0)
            }
            
            self.get_logger().info(f'Quadrant point counts: {quadrant_counts}')
            self.get_logger().info(f'Camera position in grid: Row={camera_grid_z}, Col={camera_grid_x}')
            self.get_logger().info(f'Camera position in physical: X=0.0, Z=0.0')
            self.get_logger().info(f'Map bottom-left corner: X={adjusted_min_x:.2f}, Z={adjusted_min_z:.2f}')
            self.get_logger().info(f'Map top-right corner: X={adjusted_max_x:.2f}, Z={adjusted_max_z:.2f}')
            self.get_logger().info(f'Grid map generated: {occupied_count} occupied cells')
            
            return occupancy_grid, adjusted_min_x, adjusted_min_z
            
        except Exception as e:
            self.get_logger().error(f'Error generating grid map: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            return None, 0.0, 0.0

    def trigger_map_save(self):
        """触发地图保存"""
        self.get_logger().info('Triggering map save...')
        
        # 筛选点云数据
        filtered_points = self.filter_points_by_height()
        
        if len(filtered_points) == 0:
            self.get_logger().error('No points in height range after filtering, cannot save map')
            return
        
        # 生成栅格地图
        occupancy_grid, min_x, min_z = self.generate_grid_map(filtered_points)
        
        if occupancy_grid is None:
            self.get_logger().error('Failed to generate grid map')
            return
        
        # 保存地图
        success = self.save_map_files(occupancy_grid, min_x, min_z)
        if success:
            self.get_logger().info('Map saved successfully!')
        else:
            self.get_logger().error('Failed to save map!')

    def save_map_files(self, occupancy_grid, origin_x, origin_z):
        """保存地图文件 - 使用正确计算的原点"""
        try:
            height, width = occupancy_grid.shape
            
            self.get_logger().info(f"Map info: {width}x{height}, cell size: {self.cell_size}")
            self.get_logger().info(f"Actual origin: X={origin_x:.3f}, Z={origin_z:.3f}")
            
            # 写入 PGM 文件
            pgm_path = os.path.join(self.output_dir, 'map2.pgm')
            # 反转图像 (PGM格式中0是黑色，255是白色，但我们希望障碍物是黑色)
            pgm_image = (1 - occupancy_grid) * 255
            cv2.imwrite(pgm_path, pgm_image)
            self.get_logger().info(f'PGM map saved to {pgm_path}')

            # 写入 YAML 文件 - 使用正确计算的原点，确保标准格式
            yaml_path = os.path.join(self.output_dir, 'map2.yaml')
            
            # 手动写入YAML格式，确保标准格式
            with open(yaml_path, 'w') as f:
                f.write(f"image: {os.path.basename(pgm_path)}\n")
                f.write(f"resolution: {self.cell_size}\n")
                f.write("origin: [{:.6f}, {:.6f}, {:.1f}]\n".format(
                    float(origin_x), 
                    float(origin_z), 
                    0.0
                ))
                f.write(f"negate: 0\n")
                f.write(f"occupied_thresh: 0.65\n")
                f.write(f"free_thresh: 0.25\n")
        
            self.get_logger().info(f'YAML metadata saved to {yaml_path}')
            
            # 转换为JSON格式
            self.convert_map_to_json('map2', occupancy_grid, origin_x, origin_z)
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error saving map files: {e}")
            import traceback
            traceback.print_exc()
            return False

    def convert_map_to_json(self, map_name, occupancy_grid, origin_x, origin_z):
        """转换地图为JSON格式，并添加坐标信息"""
        json_path = os.path.join(self.output_dir, f"{map_name}.json")
        
        try:
            self.get_logger().info(f"Converting to JSON format...")
            
            # 将numpy数组转换为Python列表
            grid_list = occupancy_grid.tolist()
            
            # 统计各类单元格数量
            flat_data = [item for sublist in grid_list for item in sublist]
            free_count = flat_data.count(0)
            occupied_count = flat_data.count(1)
            
            self.get_logger().info(f"Map statistics: Free={free_count}, Occupied={occupied_count}")
            
            # 计算相机在JSON网格中的位置（考虑Z方向反转）
            map_height = len(grid_list)
            map_width = len(grid_list[0]) if grid_list else 0
            camera_grid_x = int((0 - origin_x) / self.cell_size)
            camera_grid_z = map_height - 1 - int((0 - origin_z) / self.cell_size)
            
            # 计算地图覆盖的物理范围
            physical_width = map_width * self.cell_size
            physical_height = map_height * self.cell_size
            
            # 创建所需的JSON格式，添加坐标信息
            with open(json_path, 'w') as f:
                f.write("# Camera position in grid: Row={}, Col={}\n".format(camera_grid_z, camera_grid_x))
                f.write("# Map origin: X={:.3f}, Z={:.3f}\n".format(origin_x, origin_z))
                f.write("# Resolution: {}\n".format(self.cell_size))
                f.write("# Physical map size: {:.2f}m x {:.2f}m\n".format(physical_width, physical_height))
                f.write("# Coordinate system: X (left-right, positive right), Z (bottom-up, positive up)\n")
                f.write("# Map covers all quadrants:\n")
                f.write("#   Q1 (X+, Z+): Top-right quadrant\n")
                f.write("#   Q2 (X-, Z+): Top-left quadrant\n")
                f.write("#   Q3 (X-, Z-): Bottom-left quadrant\n")
                f.write("#   Q4 (X+, Z-): Bottom-right quadrant\n")
                f.write("# JSON rows: from bottom (row 0) to top (row {})\n".format(map_height-1))
                f.write("grid = [\n")
                for i, row in enumerate(grid_list):
                    f.write(f"\t{str(row)}")
                    if i < len(grid_list) - 1:
                        f.write(",\n")
                    else:
                        f.write("\n")
                f.write("]")
            
            self.get_logger().info(f"SUCCESS: Map data converted and saved to {json_path}")
            self.get_logger().info(f"Map dimensions: {len(grid_list)} rows x {len(grid_list[0])} columns")
            self.get_logger().info(f"Camera position in JSON: Row={camera_grid_z}, Col={camera_grid_x}")
            self.get_logger().info(f"Physical map size: {physical_width:.2f}m x {physical_height:.2f}m")
            self.get_logger().info(f"JSON format: Your specified format with 0 and 1")

        except Exception as e:
            self.get_logger().error(f"JSON CONVERSION FAILED: {e}")
            import traceback
            traceback.print_exc()

    def cleanup_and_exit(self):
        """清理资源并退出程序"""
        if self.shutting_down:
            return
            
        self.shutting_down = True
        self.get_logger().info('Starting cleanup and exit process...')
        
        # 取消定时器
        if hasattr(self, 'timer') and self.timer:
            self.timer.cancel()
            self.get_logger().info('Timer cancelled')
        
        # 停止进程
        self.stop_processes()
        
        # 销毁节点
        self.get_logger().info('Destroying node...')
        self.destroy_node()
        
        # 标记程序可以退出
        self.get_logger().info('Cleanup complete. Exiting program...')
        
        # 强制退出程序
        os._exit(0)

    def stop_processes(self):
        """停止进程"""
        try:
            if self.rtabmap_process:
                self.get_logger().info('Stopping RTAB-Map...')
                try:
                    # 发送SIGTERM到进程组
                    os.killpg(os.getpgid(self.rtabmap_process.pid), signal.SIGTERM)
                    # 等待进程结束
                    self.rtabmap_process.wait(timeout=3)
                    self.get_logger().info('RTAB-Map stopped gracefully')
                except (subprocess.TimeoutExpired, ProcessLookupError):
                    self.get_logger().warning('RTAB-Map did not stop gracefully, forcing...')
                    try:
                        os.killpg(os.getpgid(self.rtabmap_process.pid), signal.SIGKILL)
                        self.rtabmap_process.wait(timeout=2)
                        self.get_logger().info('RTAB-Map forced to stop')
                    except:
                        self.get_logger().error('Failed to force stop RTAB-Map')
                
            if self.camera_process:
                self.get_logger().info('Stopping camera...')
                try:
                    # 发送SIGTERM到进程组
                    os.killpg(os.getpgid(self.camera_process.pid), signal.SIGTERM)
                    # 等待进程结束
                    self.camera_process.wait(timeout=3)
                    self.get_logger().info('Camera stopped gracefully')
                except (subprocess.TimeoutExpired, ProcessLookupError):
                    self.get_logger().warning('Camera did not stop gracefully, forcing...')
                    try:
                        os.killpg(os.getpgid(self.camera_process.pid), signal.SIGKILL)
                        self.camera_process.wait(timeout=2)
                        self.get_logger().info('Camera forced to stop')
                    except:
                        self.get_logger().error('Failed to force stop camera')
                
        except Exception as e:
            self.get_logger().error(f'Error stopping processes: {e}')

    def destroy_node(self):
        """重写destroy_node以确保清理资源"""
        try:
            super().destroy_node()
        except:
            pass  # 忽略任何销毁节点的错误

def main(args=None):
    # 设置信号处理
    def signal_handler(sig, frame):
        print('\nReceived interrupt signal, shutting down...')
        sys.exit(0)
        
    signal.signal(signal.SIGINT, signal_handler)
    
    # 初始化ROS
    rclpy.init(args=args)
    
    node = None
    try:
        node = AutoSLAM()
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print('User interrupted program')
    except Exception as e:
        print(f'Program error: {e}')
    finally:
        # 确保清理资源
        if node is not None and not node.shutting_down:
            node.cleanup_and_exit()
        
        # 确保ROS关闭
        if rclpy.ok():
            rclpy.shutdown()
        
        print("Program has exited successfully.")

if __name__ == '__main__':
    main()