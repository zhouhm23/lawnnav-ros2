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
import cv2
import threading
import tkinter as tk
from tkinter import ttk, messagebox
from PIL import Image, ImageTk

class AutoSLAM(Node):
    def __init__(self, gui_callback=None):
        super().__init__('auto_slam')
        
        self.gui_callback = gui_callback  # GUI回调函数
        
        # 获取当前脚本所在目录
        self.script_dir = os.path.dirname(os.path.abspath(__file__))
        self.output_dir = os.path.join(self.script_dir, 'maps')
        os.makedirs(self.output_dir, exist_ok=True)
        
        # 参数
        self.cell_size = 0.05  # 5cm栅格大小
        self.height_min = -0.2  # 最小高度-20cm
        self.height_max = 0.0   # 最大高度0cm
        # NOTE: 不在此处显式管理 RTAB-Map 的 .db 文件，使用 RTAB-Map 默认的保存位置（例如 /home/ubuntu/.ros/rtabmap.db）
        
        # 进程和状态
        self.rtabmap_process = None
        self.camera_process = None
        self.map_saved = False
        self.shutting_down = False
        self.is_running = False
        
        # 点云数据
        self.pointcloud_data = []
        self.pointcloud_received = False
        
        # 订阅点云话题
        self.subscription = self.create_subscription(
            PointCloud2,
            '/rtabmap/cloud_map',
            self.pointcloud_callback,
            10)
        
        self.get_logger().info('Auto SLAM system initialized, waiting for GUI command...')

    def start_system(self):
        """启动相机和RTAB-Map"""
        if self.is_running:
            self.get_logger().warning('System is already running!')
            return False
            
        try:
            self.is_running = True
            self.map_saved = False
            self.pointcloud_data = []
            self.pointcloud_received = False
            
            # 启动相机节点
            camera_script = os.path.join(self.script_dir, 'start3.py')
            if os.path.exists(camera_script):
                self.get_logger().info('Starting camera node...')
                if self.gui_callback:
                    self.gui_callback("启动相机节点...")
                
                self.camera_process = subprocess.Popen(
                    [sys.executable, camera_script],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    preexec_fn=os.setsid
                )
                time.sleep(3)  # 等待相机启动
            
            # 启动RTAB-Map
            self.get_logger().info('Starting RTAB-Map...')
            if self.gui_callback:
                self.gui_callback("启动RTAB-Map建图系统...")

            # 启动 RTAB-Map（保留 --delete_db_on_start，使每次建图前删除旧数据库）
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
            if self.gui_callback:
                self.gui_callback("系统启动完成，开始建图...")
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to start system: {e}')
            if self.gui_callback:
                self.gui_callback(f"启动失败: {e}")
            self.is_running = False
            return False

    def stop_system(self):
        """停止系统并保存地图"""
        if not self.is_running:
            self.get_logger().warning('System is not running!')
            return False
            
        self.get_logger().info('Stopping system and saving map...')
        if self.gui_callback:
            self.gui_callback("正在停止系统并保存地图...")
        
        # 触发地图保存
        success = self.trigger_map_save()
        
        # 清理进程
        self.cleanup_processes()
        
        self.is_running = False
        
        if success:
            if self.gui_callback:
                self.gui_callback("地图保存完成！")
            return True
        else:
            if self.gui_callback:
                self.gui_callback("地图保存失败！")
            return False

    def pointcloud_callback(self, msg):
        """点云回调函数"""
        if not self.is_running or self.shutting_down:
            return
            
        try:
            if not self.pointcloud_received:
                self.get_logger().info('First pointcloud received!')
                if self.gui_callback:
                    self.gui_callback("接收到点云数据，开始建图...")
                self.pointcloud_received = True
            
            # 提取点云数据
            points_gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            
            new_points = []
            for point in points_gen:
                if len(point) == 3:
                    new_points.append(point)
            
            self.pointcloud_data.extend(new_points)
            
            # 更新状态信息
            if len(self.pointcloud_data) % 1000 == 0 and self.gui_callback:
                self.gui_callback(f"已采集 {len(self.pointcloud_data)} 个点云数据点")
            
        except Exception as e:
            self.get_logger().error(f'Error processing pointcloud: {e}')

    def filter_points_by_height(self):
        """筛选高度在-20cm到0cm的点"""
        if not self.pointcloud_data:
            self.get_logger().warning('No pointcloud data to filter')
            return []
        
        try:
            self.get_logger().info(f'Total points before filtering: {len(self.pointcloud_data)}')
            if self.gui_callback:
                self.gui_callback(f"过滤点云数据: 总共 {len(self.pointcloud_data)} 个点")
            
            filtered_points = []
            for point in self.pointcloud_data:
                if len(point) == 3:
                    x, y, z = point
                    # 使用Y坐标作为高度进行过滤
                    if self.height_min <= y <= self.height_max:
                        filtered_points.append((x, y, z))
            
            self.get_logger().info(f'Points after height filtering: {len(filtered_points)}')
            if self.gui_callback:
                self.gui_callback(f"高度过滤完成: {len(filtered_points)} 个点")
            
            return filtered_points
            
        except Exception as e:
            self.get_logger().error(f'Error filtering points by height: {e}')
            return []

    def generate_grid_map(self, points):
        """生成栅格地图"""
        if len(points) == 0:
            self.get_logger().warning('No points to generate grid map')
            return None, 0.0, 0.0
        
        try:
            # 提取x和z坐标
            x_coords = [p[0] for p in points]
            z_coords = [p[2] for p in points]
            
            # 计算点云边界
            min_x, max_x = min(x_coords), max(x_coords)
            min_z, max_z = min(z_coords), max(z_coords)
            
            # 扩展地图边界
            map_margin_x = 2.0
            map_margin_z = 2.0
            adjusted_min_x = min(min_x, -map_margin_x)
            adjusted_min_z = min(min_z, -map_margin_z)
            adjusted_max_x = max(max_x, map_margin_x)
            adjusted_max_z = max(max_z, map_margin_z)
            
            # 计算地图尺寸
            map_width = max(1, int((adjusted_max_x - adjusted_min_x) / self.cell_size) + 1)
            map_height = max(1, int((adjusted_max_z - adjusted_min_z) / self.cell_size) + 1)
            
            if self.gui_callback:
                self.gui_callback(f"生成栅格地图: {map_width}x{map_height}")
            
            # 创建地图
            occupancy_grid = np.zeros((map_height, map_width), dtype=np.uint8)
            
            # 将点云投影到栅格地图
            occupied_count = 0
            for x, y, z in points:
                grid_x = int((x - adjusted_min_x) / self.cell_size)
                grid_z = map_height - 1 - int((z - adjusted_min_z) / self.cell_size)
                
                if 0 <= grid_x < map_width and 0 <= grid_z < map_height:
                    occupancy_grid[grid_z, grid_x] = 1
                    occupied_count += 1
            
            self.get_logger().info(f'Grid map generated: {occupied_count} occupied cells')
            if self.gui_callback:
                self.gui_callback(f"地图生成完成: {occupied_count} 个占用栅格")
            
            return occupancy_grid, adjusted_min_x, adjusted_min_z
            
        except Exception as e:
            self.get_logger().error(f'Error generating grid map: {e}')
            return None, 0.0, 0.0

    def trigger_map_save(self):
        """触发地图保存"""
        self.get_logger().info('Triggering map save...')
        if self.gui_callback:
            self.gui_callback("正在保存地图...")
        
        # 筛选点云数据
        filtered_points = self.filter_points_by_height()
        
        if len(filtered_points) == 0:
            self.get_logger().error('No points in height range after filtering')
            if self.gui_callback:
                self.gui_callback("错误: 没有找到符合条件的点云数据")
            return False
        
        # 生成栅格地图
        occupancy_grid, min_x, min_z = self.generate_grid_map(filtered_points)
        
        if occupancy_grid is None:
            self.get_logger().error('Failed to generate grid map')
            return False
        
        # 保存地图
        success = self.save_map_files(occupancy_grid, min_x, min_z)
        return success

    def save_map_files(self, occupancy_grid, origin_x, origin_z):
        """保存地图文件"""
        try:
            height, width = occupancy_grid.shape
            
            self.get_logger().info(f"Saving map: {width}x{height}")
            if self.gui_callback:
                self.gui_callback(f"保存地图文件: {width}x{height} 栅格")
            
            # 写入 PGM 文件
            pgm_path = os.path.join(self.output_dir, 'map2.pgm')
            pgm_image = (1 - occupancy_grid) * 255
            cv2.imwrite(pgm_path, pgm_image)
            
            # 写入 YAML 文件
            yaml_path = os.path.join(self.output_dir, 'map2.yaml')
            with open(yaml_path, 'w') as f:
                f.write(f"image: {os.path.basename(pgm_path)}\n")
                f.write(f"resolution: {self.cell_size}\n")
                f.write("origin: [{:.6f}, {:.6f}, {:.1f}]\n".format(
                    float(origin_x), float(origin_z), 0.0))
                f.write(f"negate: 0\n")
                f.write(f"occupied_thresh: 0.65\n")
                f.write(f"free_thresh: 0.25\n")
            
            # 转换为JSON格式
            self.convert_map_to_json('map2', occupancy_grid, origin_x, origin_z)
            
            self.get_logger().info('Map files saved successfully')
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error saving map files: {e}")
            return False

    def convert_map_to_json(self, map_name, occupancy_grid, origin_x, origin_z):
        """转换地图为JSON格式"""
        json_path = os.path.join(self.output_dir, f"{map_name}.json")
        
        try:
            grid_list = occupancy_grid.tolist()
            map_height = len(grid_list)
            map_width = len(grid_list[0]) if grid_list else 0
            
            # 计算相机位置
            camera_grid_x = int((0 - origin_x) / self.cell_size)
            camera_grid_z = map_height - 1 - int((0 - origin_z) / self.cell_size)
            
            with open(json_path, 'w') as f:
                f.write("# Camera position in grid: Row={}, Col={}\n".format(camera_grid_z, camera_grid_x))
                f.write("# Map origin: X={:.3f}, Z={:.3f}\n".format(origin_x, origin_z))
                f.write("# Resolution: {}\n".format(self.cell_size))
                f.write("# Coordinate system: X (left-right, positive right), Z (bottom-up, positive up)\n")
                f.write("grid = [\n")
                for i, row in enumerate(grid_list):
                    f.write(f"\t{str(row)}")
                    if i < len(grid_list) - 1:
                        f.write(",\n")
                    else:
                        f.write("\n")
                f.write("]")
            
            self.get_logger().info(f"JSON map saved to {json_path}")
            if self.gui_callback:
                self.gui_callback(f"JSON地图已保存: {map_width}x{map_height}")
            
        except Exception as e:
            self.get_logger().error(f"JSON conversion failed: {e}")

    def cleanup_processes(self):
        """清理进程"""
        try:
            if self.rtabmap_process:
                self.get_logger().info('Stopping RTAB-Map (graceful SIGINT)...')
                try:
                    # 先尝试优雅结束，发送 SIGINT 让 RTAB-Map 有机会将数据库写盘
                    os.killpg(os.getpgid(self.rtabmap_process.pid), signal.SIGINT)
                    self.rtabmap_process.wait(timeout=5)
                except Exception:
                    try:
                        # SIGINT 未结束时退回到 SIGTERM
                        self.get_logger().info('SIGINT failed, sending SIGTERM')
                        os.killpg(os.getpgid(self.rtabmap_process.pid), signal.SIGTERM)
                        self.rtabmap_process.wait(timeout=3)
                    except Exception:
                        try:
                            # 最后强制杀死
                            self.get_logger().warning('SIGTERM failed, sending SIGKILL')
                            os.killpg(os.getpgid(self.rtabmap_process.pid), signal.SIGKILL)
                            self.rtabmap_process.wait(timeout=2)
                        except Exception:
                            pass
                
            if self.camera_process:
                self.get_logger().info('Stopping camera...')
                try:
                    os.killpg(os.getpgid(self.camera_process.pid), signal.SIGTERM)
                    self.camera_process.wait(timeout=3)
                except:
                    try:
                        os.killpg(os.getpgid(self.camera_process.pid), signal.SIGKILL)
                        self.camera_process.wait(timeout=2)
                    except:
                        pass
                
        except Exception as e:
            self.get_logger().error(f'Error stopping processes: {e}')

    def shutdown(self):
        """关闭节点"""
        self.shutting_down = True
        if self.is_running:
            self.cleanup_processes()
        self.destroy_node()


class SLAMGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Auto SLAM 控制系统")
        self.root.geometry("800x600")
        
        # SLAM节点
        self.slam_node = None
        self.ros_thread = None
        self.running = False
        
        self.setup_gui()
        
        # 绑定关闭事件
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
    def setup_gui(self):
        """设置GUI界面"""
        # 主框架
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 标题
        title_label = ttk.Label(main_frame, text="Auto SLAM 控制系统", 
                               font=("Arial", 16, "bold"))
        title_label.grid(row=0, column=0, columnspan=2, pady=(0, 20))
        
        # 控制按钮框架
        button_frame = ttk.Frame(main_frame)
        button_frame.grid(row=1, column=0, columnspan=2, pady=(0, 20))
        
        self.start_button = ttk.Button(button_frame, text="开始建图", 
                                      command=self.start_slam, width=15)
        self.start_button.grid(row=0, column=0, padx=(0, 10))
        
        self.stop_button = ttk.Button(button_frame, text="停止并保存", 
                                     command=self.stop_slam, width=15, state="disabled")
        self.stop_button.grid(row=0, column=1, padx=(10, 0))
        
        # 状态显示
        status_frame = ttk.LabelFrame(main_frame, text="系统状态", padding="10")
        status_frame.grid(row=2, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 20))
        
        self.status_text = tk.Text(status_frame, height=8, width=70, state="disabled")
        scrollbar = ttk.Scrollbar(status_frame, orient="vertical", command=self.status_text.yview)
        self.status_text.configure(yscrollcommand=scrollbar.set)
        
        self.status_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))
        
        # 信息显示
        info_frame = ttk.LabelFrame(main_frame, text="系统信息", padding="10")
        info_frame.grid(row=3, column=0, columnspan=2, sticky=(tk.W, tk.E))
        
        info_text = """
使用说明:
1. 点击"开始建图"启动SLAM系统
2. 移动设备进行环境扫描
3. 点击"停止并保存"结束建图并保存地图
4. 地图文件将保存在当前目录的maps文件夹中

地图格式:
- map2.pgm: 栅格地图图像
- map2.yaml: 地图配置文件  
- map2.json: JSON格式地图数据
        """
        
        info_label = ttk.Label(info_frame, text=info_text, justify=tk.LEFT)
        info_label.grid(row=0, column=0, sticky=tk.W)
        
        # 配置网格权重
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(0, weight=1)
        main_frame.rowconfigure(2, weight=1)
        status_frame.columnconfigure(0, weight=1)
        status_frame.rowconfigure(0, weight=1)
        
    def update_status(self, message):
        """更新状态显示"""
        def _update():
            self.status_text.configure(state="normal")
            self.status_text.insert(tk.END, f"{time.strftime('%H:%M:%S')} - {message}\n")
            self.status_text.see(tk.END)
            self.status_text.configure(state="disabled")
            self.root.update()
        
        self.root.after(0, _update)
        
    def start_slam(self):
        """启动SLAM系统"""
        if self.running:
            messagebox.showwarning("警告", "系统已经在运行中！")
            return
            
        self.running = True
        self.start_button.config(state="disabled")
        self.stop_button.config(state="normal")
        
        self.update_status("正在初始化SLAM系统...")
        
        # 在新线程中启动ROS2节点
        self.ros_thread = threading.Thread(target=self.start_ros_node)
        self.ros_thread.daemon = True
        self.ros_thread.start()
        
    def stop_slam(self):
        """停止SLAM系统"""
        if not self.running:
            messagebox.showwarning("警告", "系统未在运行！")
            return
            
        self.update_status("正在停止SLAM系统...")
        self.stop_button.config(state="disabled")
        
        if self.slam_node:
            success = self.slam_node.stop_system()
            if success:
                self.update_status("SLAM系统已成功停止")
            else:
                self.update_status("SLAM系统停止过程中出现错误")
        
        self.running = False
        self.start_button.config(state="normal")
        
    def start_ros_node(self):
        """启动ROS2节点"""
        try:
            # 初始化ROS2
            rclpy.init()
            
            # 创建SLAM节点
            self.slam_node = AutoSLAM(gui_callback=self.update_status)
            
            # 启动系统
            success = self.slam_node.start_system()
            
            if not success:
                self.update_status("系统启动失败！")
                self.running = False
                self.root.after(0, lambda: self.start_button.config(state="normal"))
                self.root.after(0, lambda: self.stop_button.config(state="disabled"))
                return
            
            # 运行ROS2节点
            self.update_status("ROS2节点开始运行...")
            rclpy.spin(self.slam_node)
            
        except Exception as e:
            self.update_status(f"ROS2节点错误: {e}")
            self.running = False
            self.root.after(0, lambda: self.start_button.config(state="normal"))
            self.root.after(0, lambda: self.stop_button.config(state="disabled"))
            
    def on_closing(self):
        """关闭窗口时的处理"""
        if self.running:
            if messagebox.askokcancel("退出", "系统正在运行，确定要退出吗？"):
                if self.slam_node:
                    self.slam_node.shutdown()
                if rclpy.ok():
                    rclpy.shutdown()
                self.root.destroy()
        else:
            if self.slam_node:
                self.slam_node.shutdown()
            if rclpy.ok():
                rclpy.shutdown()
            self.root.destroy()


def main():
    # 创建GUI
    root = tk.Tk()
    app = SLAMGUI(root)
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        print("程序被用户中断")
    finally:
        # 清理资源
        if app.slam_node:
            app.slam_node.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()