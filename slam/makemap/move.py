"""GUI 控制小车运动：按住按钮持续发布速度到 `/controller/cmd_vel`。

使用方法：运行此脚本会弹出窗口，按住按钮时持续发送对应速度。
"""
import threading
import os
import tkinter as tk
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MoveNode(Node):
    def __init__(self, topic_name='/controller/cmd_vel', linear_speed=0.03, angular_speed=0.1):
        super().__init__('move_gui_node')
        self.cmd_topic = topic_name
        self.get_logger().info(f'Publishing cmd topic: {self.cmd_topic}')
        # 创建主发布者
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        # 同时创建一个到 /cmd_vel 的发布者（若主话题本来就是 /cmd_vel 会共享同一个话题）
        if self.cmd_topic != '/cmd_vel':
            self.cmd_vel_fallback_pub = self.create_publisher(Twist, '/cmd_vel', 10)
            self.get_logger().info('Also publishing to /cmd_vel for compatibility')
        else:
            self.cmd_vel_fallback_pub = None
        # 当前动作由 GUI 线程更新；timer 在 rclpy spin 中运行并发布对应速度
        self._lock = threading.Lock()
        self.current_action = None
        # 基线速度（可以由 GUI 提供）
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed
        # 订阅当前 cmd topic（用于读取基线或其它节点发布的速度）
        try:
            self.subscription = self.create_subscription(Twist, self.cmd_topic, self._cmd_callback, 10)
        except Exception:
            # 订阅失败时继续运行（某些话题在创建时可能不存在）
            self.subscription = None
        self.latest_cmd = Twist()
        # 使用与 auto_mower 类似的控制周期（0.1s）发布速度
        self.create_timer(0.1, self._control_timer)

    def publish_cmd(self, linear_x: float = 0.0, angular_z: float = 0.0):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        # 发布到主话题
        try:
            self.cmd_vel_pub.publish(msg)
        except Exception:
            pass
        # 同时发布到 /cmd_vel（如果存在备用发布者）
        if getattr(self, 'cmd_vel_fallback_pub', None) is not None:
            try:
                self.cmd_vel_fallback_pub.publish(msg)
            except Exception:
                pass

    def _cmd_callback(self, msg: Twist):
        # 存储最近一次收到的 Twist，供定时器参考
        with self._lock:
            self.latest_cmd = msg

    def set_action(self, action):
        with self._lock:
            self.current_action = action

    def _control_timer(self):
        # 周期性在 ROS 定时器中发布，保证由 rclpy 管理
        with self._lock:
            action = self.current_action

        if action is None:
            # 发送零速度以确保稳定停止
            self.publish_cmd(0.0, 0.0)
            return

        # 基于 latest_cmd 作为基线，替换需要的分量
        with self._lock:
            base = self.latest_cmd

        lin_x = base.linear.x
        lin_y = base.linear.y
        ang_z = base.angular.z

        if action == 'forward':
            lin_x = self.linear_speed
        elif action == 'back':
            lin_x = -self.linear_speed
        elif action == 'cw':
            ang_z = -abs(self.angular_speed)
        elif action == 'ccw':
            ang_z = abs(self.angular_speed)

        # 对于麦克纳姆底盘，保留 linear.y 不变（可在未来映射为左右移动）
        self.publish_cmd(lin_x, ang_z)


class MoveGUI:
    def __init__(self, linear_speed=0.1, angular_speed=0.15, publish_interval_ms=100, topic=None):
        # 先保存速度参数，这样可以传递给 MoveNode 保持一致
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed
        self.interval = publish_interval_ms

        # 初始化 ROS
        rclpy.init()
        if topic is None:
            topic = os.environ.get('CMD_TOPIC', '/controller/cmd_vel')
        # 将速度传给 MoveNode，保证 node 的基线速度与 GUI 一致
        self.node = MoveNode(topic_name=topic, linear_speed=self.linear_speed, angular_speed=self.angular_speed)

        # 在后台线程 spin，保证 rclpy 可用
        self._spin_thread = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self._spin_thread.start()

        # GUI 状态
        self.current_action = None  # 'forward', 'back', 'cw', 'ccw' or None
        # tk.after 的 id（用于取消）
        self._after_id = None

        # 创建窗口（tk 必须在主线程）
        self.root = tk.Tk()
        self.root.title('Move Control')
        self.root.protocol('WM_DELETE_WINDOW', self._on_close)

        # 布局按钮
        btn_fw = tk.Button(self.root, text='前进', width=10)
        btn_bw = tk.Button(self.root, text='后退', width=10)
        btn_ccw = tk.Button(self.root, text='逆时针', width=10)
        btn_cw = tk.Button(self.root, text='顺时针', width=10)

        btn_fw.grid(row=0, column=1, padx=6, pady=6)
        btn_ccw.grid(row=1, column=0, padx=6, pady=6)
        btn_cw.grid(row=1, column=2, padx=6, pady=6)
        btn_bw.grid(row=2, column=1, padx=6, pady=6)

        # 绑定按下/释放事件；只有按住才持续执行
        btn_fw.bind('<ButtonPress-1>', lambda e: self._start_action('forward'))
        btn_fw.bind('<ButtonRelease-1>', lambda e: self._stop_action())

        btn_bw.bind('<ButtonPress-1>', lambda e: self._start_action('back'))
        btn_bw.bind('<ButtonRelease-1>', lambda e: self._stop_action())

        btn_cw.bind('<ButtonPress-1>', lambda e: self._start_action('cw'))
        btn_cw.bind('<ButtonRelease-1>', lambda e: self._stop_action())

        btn_ccw.bind('<ButtonPress-1>', lambda e: self._start_action('ccw'))
        btn_ccw.bind('<ButtonRelease-1>', lambda e: self._stop_action())

        # 防止在按钮外释放鼠标导致未捕获释放事件：在根窗口也监听释放
        self.root.bind('<ButtonRelease-1>', lambda e: self._stop_action())

    def _start_action(self, action_name: str):
        # 如果正在执行相同动作，忽略
        if self.current_action == action_name:
            return
        # 取消可能残留的重复发布
        if self._after_id is not None:
            try:
                self.root.after_cancel(self._after_id)
            except Exception:
                pass
            self._after_id = None

        self.current_action = action_name
        # 告知 ROS 节点开始该动作
        try:
            self.node.set_action(action_name)
        except Exception:
            pass

        # 立即发送一次以降低延迟，然后启动周期性发布（tk 的主线程负责）
        try:
            if action_name == 'forward':
                self.node.publish_cmd(self.linear_speed, 0.0)
            elif action_name == 'back':
                self.node.publish_cmd(-self.linear_speed, 0.0)
            elif action_name == 'cw':
                self.node.publish_cmd(0.0, -abs(self.angular_speed))
            elif action_name == 'ccw':
                self.node.publish_cmd(0.0, abs(self.angular_speed))
        except Exception:
            pass

        # 启动周期性发布，确保即使 node 内部状态不同步也能持续发送
        self._after_id = self.root.after(self.interval, self._repeat_publish)

    def _stop_action(self):
        if self.current_action is None:
            return
        self.current_action = None
        # 取消周期性发布
        if self._after_id is not None:
            try:
                self.root.after_cancel(self._after_id)
            except Exception:
                pass
            self._after_id = None

        # 告知节点停止动作，并立即发送停止速度
        try:
            self.node.set_action(None)
        except Exception:
            pass
        try:
            self.node.publish_cmd(0.0, 0.0)
        except Exception:
            pass

    def _publish(self, linear, angular):
        try:
            self.node.publish_cmd(linear, angular)
        except Exception:
            # 在极少数情况下后台spin可能已结束
            pass
    def _repeat_publish(self):
        # 根据当前 action 重复发送命令，并重新安排下一次调用
        action = self.current_action
        try:
            if action == 'forward':
                self.node.publish_cmd(self.linear_speed, 0.0)
            elif action == 'back':
                self.node.publish_cmd(-self.linear_speed, 0.0)
            elif action == 'cw':
                self.node.publish_cmd(0.0, -abs(self.angular_speed))
            elif action == 'ccw':
                self.node.publish_cmd(0.0, abs(self.angular_speed))
            else:
                # 如果没有动作，发送停止并返回
                try:
                    self.node.publish_cmd(0.0, 0.0)
                except Exception:
                    pass
                self._after_id = None
                return
        except Exception:
            pass

        # 继续循环
        try:
            self._after_id = self.root.after(self.interval, self._repeat_publish)
        except Exception:
            self._after_id = None
    

    def _on_close(self):
        # 停止动作并清理 ROS
        self._stop_action()
        try:
            # 销毁 node 并 shutdown rclpy
            self.node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        # 等待 spin 线程结束（它是 daemon，一般会随进程退出）
        try:
            if self._spin_thread.is_alive():
                self._spin_thread.join(timeout=0.5)
        except Exception:
            pass
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main():
    # 你可以通过环境变量 CMD_TOPIC 指定发布的话题，例如: CMD_TOPIC=/cmd_vel
    gui = MoveGUI()
    gui.run()


if __name__ == '__main__':
    main()
