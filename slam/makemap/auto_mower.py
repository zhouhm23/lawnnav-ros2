import sys
import rclpy # 导入ROS2 Python客户端库
from rclpy.node import Node # 导入ROS2节点基类
from geometry_msgs.msg import Twist # 导入速度命令消息
from nav_msgs.msg import Odometry   # 导入里程计消息
from . import make_path
from .zhangaiwu import ObstacleDetector  # 导入避障检测器
import math

class AutoMowerNode(Node):
    def __init__(self):
        super().__init__('auto_mower_node')

        # 基础参数配置
        # 车初始朝向x轴，向左为y轴，角度逆时针为正
        self.vertices = [(0, 0), (0.6, 0), (0.6, 0.4), (0, 0.4)]  # 覆盖区域顶点列表
        self.mower_width = 0.2          # 割草机切割宽度（米）
        self.distances, self.angles = make_path.get_path(self.vertices, self.mower_width)
        self.get_logger().info(f'移动距离向量：{[round(coord, 2) for coord in self.distances]} 移动角度向量：{[round(math.degrees(coord), 2) for coord in self.angles]}')
        self.base_speed = 0.2           # 基础线速度（米/秒）
        self.base_angular_speed = math.pi / 12  # 基础角速度（弧度/秒）
        self.position_tolerance = 0.05  # 位置控制容差（米）
        self.angular_tolerance = math.pi / 30  # 角度控制容差（弧度，约6°）

        # 状态管理
        self.stage = -1  # -1=初始化，0=直行，1=转弯
        self.passes = 0  # 已完成距离次数
        self.max_passes = len(self.distances)  # 最大覆盖距离次数

        self.origin = [0.0, 0.0, 0.0] # 原点
        self.last_pose = [0.0, 0.0, 0.0]  # 上一阶段起点位姿
        self.current_pose = [0.0, 0.0, 0.0]  # 当前位姿
        
        self.current_local_pose = [0.0, 0.0, 0.0] # 当前局部坐标
        self.last_local_pose = [0.0, 0.0, 0.0] # 上一局部坐标

        # 避障功能
        self.obstacle_detector = ObstacleDetector(safe_distance_cm=30)
        self.obstacle_detector.start_detection_async()
        self.get_logger().info("避障检测器已启动")

        # ROS接口
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/controller/cmd_vel', 10)
        self.create_timer(0.1, self.control_cycle)

    def quaternion_to_yaw(self, q):
        """将四元数转换为航向角（Yaw）"""
        return math.atan2(
            2 * (q.w * q.z + q.x * q.y), 
            1 - 2 * (q.y**2 + q.z**2)
        )

    def odom_callback(self, msg):
        """里程计数据回调，更新当前位姿"""
        self.current_pose = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            self.quaternion_to_yaw(msg.pose.pose.orientation)
        ]
    
    def to_local(self, world_pose, pose0):
        """将世界坐标系的位姿转换为局部坐标系"""
        dx = world_pose[0] - pose0[0]
        dy = world_pose[1] - pose0[1]
        y0 = pose0[2]  # 原点的偏航角
        x = dx * math.cos(y0) + dy * math.sin(y0)
        y = -dx * math.sin(y0) + dy * math.cos(y0)
        
        a = ((world_pose[2] - y0 + math.pi) % (2 * math.pi) - math.pi) * 180 / math.pi # 转换为度
        return x, y, a

    def compute_distance(self, pose1, pose2):
        """计算两点距离"""
        dx = pose2[0] - pose1[0]
        dy = pose2[1] - pose1[1]
        return math.sqrt(dx**2 + dy**2)

    def get_angle(self, start_angle, change_angle):
        """输入开始角度和角度变化量，计算目标角度（都是弧度制），且限制在[-π, π]范围内"""
        target_angle = start_angle + change_angle
        while target_angle > math.pi:
            target_angle -= 2 * math.pi
        while target_angle < -math.pi:
            target_angle += 2 * math.pi
        return target_angle

    def stop(self):
        """发布停止命令"""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)

    def destroy_node(self):
        """清理资源"""
        self.obstacle_detector.stop_detection()
        super().destroy_node()

    def control_cycle(self):
        """控制周期主函数（每0.1秒执行一次）"""
        cmd = Twist()

        # 阶段-1：初始化
        if self.stage == -1:
            if self.current_pose != [0.0, 0.0, 0.0]:
                self.last_pose = self.current_pose.copy()
                self.origin = self.current_pose.copy()
                self.stage = 0

        # 阶段0：直行
        elif self.stage == 0:

            if self.obstacle_detector.obstacle_detected:
                self.stop()
                self.get_logger().warn(f"检测到障碍物: {self.obstacle_detector.status_text}，暂停前进")
            else:
                cmd.linear.x = self.base_speed

            if abs(self.compute_distance(self.last_pose, self.current_pose) - self.distances[self.passes]) <= self.position_tolerance:
                self.stop()
                self.passes += 1
                self.stage = 1
                self.last_pose = self.current_pose.copy()

        # 阶段1：转弯
        elif self.stage == 1:
            target_angle = self.get_angle(self.last_pose[2], self.angles[self.passes])

            cmd.angular.z = self.base_angular_speed * math.copysign(1, self.angles[self.passes])

            if abs(target_angle - self.current_pose[2]) <= self.angular_tolerance:
                self.stop()
                self.stage = 0
                self.last_pose = self.current_pose.copy()

        # 全局判定：是否完成所有覆盖
        if self.passes >= self.max_passes:
            self.stop()
            self.obstacle_detector.stop_detection()  # 停止避障检测
            rclpy.shutdown()
            sys.exit(0)
            return

        self.cmd_vel_pub.publish(cmd)

        # 局部坐标更新
        self.current_local_pose = self.to_local(self.current_pose, self.origin)
        self.last_local_pose = self.to_local(self.last_pose, self.origin)
        # 使用 sys.stdout.write 和 sys.stdout.flush 实现覆盖输出
        sys.stdout.write(
            f'\rCurrent Local Pose: {[round(coord, 2) for coord in self.current_local_pose]} '
            f'Last Local Pose: {[round(coord, 2) for coord in self.last_local_pose]}'
        )
        sys.stdout.flush()


def main(args=None):
    rclpy.init(args=args)
    node = AutoMowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        return 0


if __name__ == '__main__':
    main()