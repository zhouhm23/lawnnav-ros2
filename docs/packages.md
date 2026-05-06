# 功能包说明

> 以下为 Git 仓库追踪的包。`app/`、`bringup/`、`driver/`、`peripherals/`、`yolov5_ros2/` 等依赖包已被 `.gitignore` 排除，不纳入版本管理。

---

## path_coverage（路径覆盖规划包）

**路径**: `src/path_coverage_ros2/`

**功能**: 基于 Boustrophedon（牛耕式）细胞分解的覆盖路径规划。用户通过 RViz 的 Publish Point 点击定义多边形区域，系统自动分解为子细胞并生成往复式扫描路径，经 Nav2 动作客户端逐点导航执行。

**核心算法**: Ruby Boustrophedon 分解 → `trapezoidal_coverage.py` 梯形往复路径生成 → Greedy TSP 细胞排序

**关键参数**: `robot_width`=0.171m, `polygon_expand`=0.05m, `min_wp_dist`=0.2m

**依赖**: `python3-shapely`, `ruby-full`, `nav2_bringup`

**启动**: `ros2 launch path_coverage path_coverage.launch.py`

**基于开源**: [path_coverage_ros2](https://github.com/nirmalka94/path_coverage_ros2/tree/main)，本项目完成 Nav2 深度联动、异步目标发送、参数调优、静态图掩膜等工程化改进。

---

## navigation（导航配置包）

**路径**: `src/navigation/`

**功能**: Nav2 导航栈的控制器/规划器参数配置与启动。包含 RTAB-Map + Nav2 联合启动文件 (`rtabmap_navigation.launch.py`) 和 RViz 可视化启动。控制器为 Regulated Pure Pursuit，全局规划器为 A*。

**关键文件**: `config/nav2_params.yaml`, `launch/navigation.launch.py`, `launch/rtabmap_navigation.launch.py`

---

## slam（SLAM 建图包）

**路径**: `src/slam/`

**功能**: RTAB-Map 视觉 SLAM 启动配置、建图脚本、地图保存、地面颜色语义叠加 (`map_color_overlay.py`)。支持 mapping/localization 模式切换。

**关键文件**: `launch/rtabmap_slam.launch.py`, `scripts/map_color_overlay.py`, `slam/map_save.py`

---

## coverage_evaluator（覆盖率评估包）

**路径**: `src/coverage_evaluator/`

**功能**: 基于 SLAM 定位数据的实时覆盖率计算。用户在 RViz 中通过 Publish Point 点击定义多边形覆盖区域，系统以 0.005m 分辨率栅格化区域，通过 tf2 监听 `map→base_footprint` 变换，以 `coverage_radius=0.12m` 为半径逐帧标记圆形覆盖区域，实时计算并发布覆盖率比值。

**注意**: 此包的覆盖率计算完全依赖机器人 SLAM 定位数据，因此结果受 SLAM 定位精度影响，存在自测偏差。论文中引入的"基于俯视摄像的真值评估系统"即为解决此偏差而设计的外部客观评测手段。

---

## radar（毫米波雷达脚本）

**路径**: `src/radar/`（非标准 ROS 包，无 `package.xml`）

**功能**: 毫米波雷达串口数据采集 (`serial_port_connector.py`) 与 Range/Doppler FFT 处理 (`radar_data_processor.py`)。

---

## tools（测试工具集）

**路径**: `src/tools/`

| 文件 | 用途 |
|------|------|
| `test1_slam_nav_test.py` | SLAM 闭合路径 RPE + 静态定位稳定性测试 |
| `test2_nav_cte_and_obstacle_test.py` | 直线跟踪 CTE + 绕障安全裕度测试 |
| `test_utils.py` | 公共工具：CSVLogger, StuckDetector, rotate_360, 坐标转换 |
| `start_path_coverage.py` | 一键启动脚本（Python 版） |
| `start_path_coverage.sh` | 一键启动脚本（Shell 兼容包装） |

---

## 已被 gitignore 的依赖包（仅供参考）

以下包在工作空间中存在但未被 Git 追踪，属于硬件驱动、外设、仿真等依赖：

| 目录 | 用途 |
|------|------|
| `driver/` | 硬件驱动层（controller, ros_robot_controller, sdk） |
| `peripherals/` | 外设驱动（手柄/键盘遥控、IMU TF） |
| `app/` | 上层应用（AR、手势控制、巡线、物体跟踪） |
| `bringup/` | 系统启动（硬件自检、服务管理） |
| `calibration/` | 运动标定（线速度/角速度里程计偏差） |
| `interfaces/` | 自定义 ROS2 消息和服务定义 |
| `yolov5_ros2/` | YOLOv5 目标检测 |
| `simulations/` | 机器人 URDF 模型描述 |
| `multi/` | 多机协同编队管理 |
| `example/` | 各功能模块示例代码 |
| `auto_mower/` | 个人学习参考包（非正式功能） |
