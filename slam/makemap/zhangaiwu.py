# zhangaiwu.py
import pyrealsense2 as rs
import numpy as np
import cv2
import threading
import time

class ObstacleDetector:
    def __init__(self, safe_distance_cm=30):
        # 参数设置
        self.SAFE_DISTANCE_CM = safe_distance_cm
        self.WIDTH = 640
        self.HEIGHT = 480
        self.FPS = 30
        
        # 障碍物检测的感兴趣区域 (ROI)
        self.ROI_START_X = 260
        self.ROI_END_X = 380
        self.ROI_START_Y = 180
        self.ROI_END_Y = 300
        
        # 局部密度检测参数
        self.WINDOW_SIZE = 20        
        self.LOCAL_INVALID_THRESHOLD = 0.90  
        
        # 初始化状态
        self.obstacle_detected = False
        self.status_text = "未初始化"
        self.running = False
        self.pipeline = None
        self.depth_scale = 0
        self.safe_dist_value = 0
        
    def initialize_camera(self):
        """初始化RealSense相机"""
        try:
            self.pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.depth, self.WIDTH, self.HEIGHT, rs.format.z16, self.FPS)
            
            profile = self.pipeline.start(config)
            depth_sensor = profile.get_device().first_depth_sensor()
            self.depth_scale = depth_sensor.get_depth_scale()
            
            self.safe_dist_value = (self.SAFE_DISTANCE_CM / 100.0) / self.depth_scale
            return True
        except Exception as e:
            print(f"启动 RealSense 失败: {e}")
            return False
    
    def check_for_obstacle_by_density(self, roi_depth):
        """
        障碍物检测逻辑，返回布尔值和详细状态文本。
        """
        
        # 检查有效深度值（条件 A：有效深度过近）
        valid_depths = roi_depth[roi_depth > 0]
        
        if valid_depths.size > 0:
            is_close_by_valid_data = np.any(valid_depths < self.safe_dist_value)
            if is_close_by_valid_data:
                min_dist_value = np.min(valid_depths)
                min_dist_cm = min_dist_value * self.depth_scale * 100
                return True, f"有效深度点过近: 最近距离 {min_dist_cm:.1f}cm"

        # 检查局部无效点密度（条件 B：视为 Min-Z 过近障碍物）
        invalid_mask = (roi_depth == 0).astype(np.uint8)
        window_area = self.WINDOW_SIZE * self.WINDOW_SIZE
        invalid_count_threshold = int(window_area * self.LOCAL_INVALID_THRESHOLD)
        
        kernel = np.ones((self.WINDOW_SIZE, self.WINDOW_SIZE), dtype=np.uint8)
        conv_result = cv2.filter2D(invalid_mask, -1, kernel, borderType=cv2.BORDER_CONSTANT)
        
        is_dense_invalid_area = np.any(conv_result > invalid_count_threshold)
        
        if is_dense_invalid_area:
            return True, "高密度无效点（Min-Z 范围）"

        # 安全
        return False, "安全"
    
    def detect_obstacles(self):
        """持续检测障碍物的主循环"""
        if not self.pipeline:
            if not self.initialize_camera():
                return
                
        self.running = True
        
        try:
            while self.running:
                frames = self.pipeline.wait_for_frames()
                depth_frame = frames.get_depth_frame()
                if not depth_frame:
                    continue

                depth_image = np.asanyarray(depth_frame.get_data())
                roi_depth = depth_image[self.ROI_START_Y:self.ROI_END_Y, self.ROI_START_X:self.ROI_END_X]

                # 执行障碍物检测
                self.obstacle_detected, self.status_text = self.check_for_obstacle_by_density(roi_depth)
                    
                # 控制检测频率
                time.sleep(0.1)
                
        except Exception as e:
            print(f"障碍物检测过程中发生错误: {e}")
        finally:
            self.stop_detection()
    
    def start_detection_async(self):
        """异步启动障碍物检测"""
        detection_thread = threading.Thread(target=self.detect_obstacles, daemon=True)
        detection_thread.start()
        return detection_thread
    
    def stop_detection(self):
        """停止障碍物检测"""
        self.running = False
        if self.pipeline:
            self.pipeline.stop()
            self.pipeline = None

def run_standalone():
    """独立运行模式"""
    # 参数设置
    SAFE_DISTANCE_CM = 30  # 安全距离阈值 (厘米)
    WIDTH = 640            # 深度流宽度
    HEIGHT = 480           # 深度流高度
    FPS = 30               # 帧率
    PRINT_INTERVAL_SEC = 0.5  # 终端信息输出间隔 (秒，避免刷屏太快)

    # 障碍物检测的感兴趣区域 (ROI)
    ROI_START_X = 260
    ROI_END_X = 380
    ROI_START_Y = 180
    ROI_END_Y = 300

    # 局部密度检测参数
    WINDOW_SIZE = 20        
    LOCAL_INVALID_THRESHOLD = 0.90  

    def check_for_obstacle_by_density(roi_depth, safe_dist_value, depth_scale):
        """
        障碍物检测逻辑，返回布尔值和详细状态文本。
        """
        
        # 检查有效深度值（条件 A：有效深度过近）
        valid_depths = roi_depth[roi_depth > 0]
        
        if valid_depths.size > 0:
            is_close_by_valid_data = np.any(valid_depths < safe_dist_value)
            if is_close_by_valid_data:
                min_dist_value = np.min(valid_depths)
                min_dist_cm = min_dist_value * depth_scale * 100
                return True, f"有效深度点过近: 最近距离 {min_dist_cm:.1f}cm"

        # 检查局部无效点密度（条件 B：视为 Min-Z 过近障碍物）
        invalid_mask = (roi_depth == 0).astype(np.uint8)
        window_area = WINDOW_SIZE * WINDOW_SIZE
        invalid_count_threshold = int(window_area * LOCAL_INVALID_THRESHOLD)
        
        kernel = np.ones((WINDOW_SIZE, WINDOW_SIZE), dtype=np.uint8)
        conv_result = cv2.filter2D(invalid_mask, -1, kernel, borderType=cv2.BORDER_CONSTANT)
        
        is_dense_invalid_area = np.any(conv_result > invalid_count_threshold)
        
        if is_dense_invalid_area:
            return True, "高密度无效点（Min-Z 范围）"

        # 安全
        return False, "安全"

    # 配置和启动相机流
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)

    print("启动 RealSense 摄像头...")
    try:
        profile = pipeline.start(config)
    except Exception as e:
        print(f"启动 RealSense 失败: {e}")
        exit()

    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()

    SAFE_DIST_VALUE = (SAFE_DISTANCE_CM / 100.0) / depth_scale
    print(f"安全距离阈值 ({SAFE_DISTANCE_CM}cm) 对应深度值: {SAFE_DIST_VALUE:.2f}")

    # 用于控制终端输出频率
    last_print_time = 0 

    try:
        while True:
            current_time = time.time()
            
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            if not depth_frame:
                continue

            depth_image = np.asanyarray(depth_frame.get_data())
            roi_depth = depth_image[ROI_START_Y:ROI_END_Y, ROI_START_X:ROI_END_X]

            # 执行障碍物检测
            is_obstructed, status_text = check_for_obstacle_by_density(roi_depth, SAFE_DIST_VALUE, depth_scale)

            # 终端信息输出
            if current_time - last_print_time >= PRINT_INTERVAL_SEC:
                if is_obstructed:
                    # 障碍物警报，使用醒目的颜色或符号
                    print(f"!!! 警报: {status_text} (距离阈值 {SAFE_DISTANCE_CM}cm)")
                    # 实际应用中：在这里发送小车停止指令
                else:
                    print(f"--- 状态: {status_text} (距离 > {SAFE_DISTANCE_CM}cm)")
                    # 实际应用中：发送小车继续前进指令
                
                last_print_time = current_time

            # 图像显示 (仅用于辅助观察)
            
            # 将深度图转换为彩色图
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET
            )
            
            # 仅在图像上绘制 ROI 矩形 (不绘制文字)
            rect_color = (0, 0, 255) if is_obstructed else (0, 255, 0)
            cv2.rectangle(
                depth_colormap, 
                (ROI_START_X, ROI_START_Y), 
                (ROI_END_X, ROI_END_Y), 
                rect_color, 
                2
            )
            
            cv2.imshow('RealSense 实时避障 (终端输出信息)', depth_colormap)

            # 按 'q' 退出循环
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        print("RealSense 摄像头已停止。")

# 保留原来的独立运行功能
if __name__ == "__main__":
    run_standalone()