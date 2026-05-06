# 开发历程与架构说明

## 核心模块说明
1. **路径覆盖规划模块**：基于开源项目 `path_coverage_ros2` 二次开发，在原项目 Boustrophedon 分解、基础路径生成能力的基础上，完成了与 Nav2 导航栈的深度联动，新增异步目标发送、超时判定、连续航点执行、参数调优、静态图掩膜等功能，核心文件为 `path_coverage_node.py`、`path_coverage_params.yaml`。
2. **SLAM 与导航模块**：基于 RTAB-Map 实现视觉增量建图，基于 Nav2 实现路径规划与运动控制，支持 Regulated Pure Pursuit 控制器、A* 全局规划器，核心文件为 `nav2_params.yaml`、`rtabmap.launch.py`、`navigation.launch.py`。
3. **地面颜色识别模块**：实现点云颜色检查、区域 RGB 打印、颜色语义叠加到地图等功能，核心文件为 `map_color_overlay.py`、`map_color_overlay.launch.py`。
4. **评估工具链模块**：实现多边形圈选、网格化覆盖率计算、实时覆盖率发布、离线分解预览等功能，核心文件为 `coverage_evaluator_node.py`、`offline_decompose_and_coverage_preview.py`。

## 迭代记录
### 阶段一：最小闭环搭建 (2025.11)
- **Commit 6ae4019**: 基于开源 `path_coverage_ros2` 框架新增路径覆盖主控节点 `path_coverage_node.py`，建立 `MapDrive` 主节点，接入 `NavigateToPose/ComputePathToPose` 动作客户端，订阅全局与局部代价地图，实现 RViz 点击区域采集、Boustrophedon 分解、路径插值、Marker 可视化与位姿结果输出功能。
- **Commit 3d16a05**: 优化路径覆盖执行逻辑，将原项目单次阻塞式等待改为带反馈回调的异步目标发送，引入导航完成态、成功态和反馈超时判定，去除固定 sleep 节拍，调整 `robot_width`、`min_wp_dist` 等关键参数。
- **Commit eb65af0**: 整合导航、SLAM、覆盖规划三大子系统，引入 Nav2 控制器/规划器参数集、RTAB-Map 启动描述、RViz 配置、地图资产与自动化建图脚本，补齐 ROS2 包清单、setup 与测试骨架，形成完整工程基线。

### 阶段二：视觉导航与覆盖规划调优 (2025.11-2025.12)
- **Commit 61c2a0e**: 打通视觉点云到 Nav2 代价地图的数据链路，将 `local/global costmap` 障碍源从 `LaserScan` 切换为 `PointCloud2`（`/rtabmap/cloud_map`），补充 `transform_tolerance`，RTAB-Map 改为增量建图模式并调整 remap（含 `imu/cloud_map`），在机器人启动链路中加入 xacro 解析与 `robot_state_publisher`。
- **Commit b787ac4**: 调整覆盖规划参数，收敛 `robot_width`、`num_points`、`min_wp_dist` 等关键参数，同步注释语义与配置解释，刷新位姿输出时间戳。
- **Commit 94594bd**: 更新导航控制器配置，控制插件统一切换为 Regulated Pure Pursuit，补齐前瞻距离、碰撞检测、旋转对齐、速度约束等参数，全局规划器启用 A* 且收紧容差，RViz 增加路径覆盖 Marker 观察通道。
- **Commit 951c900**: 修复 `path_coverage_node.py` 中超时问题，将 ActionClient 绑定到独立 `sub_node`，`send_goal/result` 的 `spin_until_future_complete` 统一在 `sub_node` 上执行，清理旧发布路径相关残留。

### 阶段三：语义感知与工程化升级 (2025.12-2026.04)
- **Commit 0c3a44c**: 初步增加地面颜色识别功能，新增地面投影/高度阈值相关参数，在导航与 SLAM 启动中挂载参数文件，增加点云颜色检查、区域 RGB 打印和地图数据结构检查工具，补充一键联动脚本 `launch_ccpp.sh`。
- **Commit 245d5a0**: 面向标准化测试进行系统级工程化升级，新增 `CoverageEvaluator` 模块（多边形圈选、网格化覆盖率计算、TF 坐标转换、实时覆盖率发布与 reset 服务），Nav2 增加 `static_layer`、调整膨胀半径与代价值衰减、限制 `allow_unknown`，RTAB-Map 启动支持 `localization/mapping` 模式切换与检测频率调优，覆盖规划新增 `polygon_expand`、`coverage_clearance`、执行阶段更严格代价阈值与静态图掩膜选项，新增离线分解预览脚本，引入毫米波雷达串口与帧处理模块，强化颜色叠加节点的 TF 健壮性、重发布与可视化参数。
- **Commit 8f871ed**: 清理非源码产物，删除误提交的临时二进制文件 `core`。

## 仓库结构

> 以下为 Git 追踪的顶层目录。`app/`、`bringup/`、`driver/`、`peripherals/`、`yolov5_ros2/` 等依赖包已被 `.gitignore` 排除，不纳入版本管理。

```
src/
├── path_coverage_ros2/     # 【路径覆盖规划】Boustrophedon 分解 + 往复路径生成 + Nav2 导航执行
├── navigation/             # 【导航配置】Nav2 控制器/规划器参数、RTAB-Map+Nav2 联合启动
├── slam/                   # 【SLAM 建图】RTAB-Map 启动配置、建图脚本、地面颜色叠加 (map_color_overlay)
├── coverage_evaluator/     # 【覆盖率评估】多边形圈选 + 栅格化 + 实时覆盖率发布
├── radar/                  # 【毫米波雷达】串口数据采集与 Range/Doppler FFT 处理
├── tools/                  # 【测试工具】SLAM/导航综合测试、CTE+避障测试、一键启动脚本
├── docs/                   # 【项目文档】
└── README.md
```

## 开源致谢
本项目路径覆盖规划核心功能基于开源项目 `path_coverage_ros2`（https://github.com/nirmalka94/path_coverage_ros2/tree/main）进行二次开发，原项目作者为 ROS1 版本 Erik Andresen、ROS2 版本 Azeez Adebayo。本项目在原项目 Boustrophedon 分解与基础路径生成能力的基础上，完成了与 Nav2 导航栈的深度联动、执行逻辑优化、参数调优及多传感器融合适配，感谢原作者的开源贡献。
