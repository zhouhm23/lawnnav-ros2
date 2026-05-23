# 使用方法

## 一键启动用户程序（交互式控制台）

```bash
python3 launcher/start.py           # 默认安静模式
python3 launcher/start.py --debug   # 调试模式
bash launcher/start.sh              # Shell 兼容包装
```

启动后进入交互式命令行：

```
> mapping                  ← 启动 SLAM 建图 + RViz，手动控车遍历区域

> save test_map            ← 保存地图副本到/home/ubuntu/.ros/maps
> load test_map            ← 切换地图
> list                     ← 查看所有地图和区域

> region my_area           ← 在 RViz 中 Publish Point 圈选新区域并保存
> coverage camera_map test_180x240   ← 纯定位覆盖
> live test_180x240        ← 建图模式下直接覆盖

> log [进程名]             ← 查看进程日志
> status                   ← 查看进程状态
> stop                     ← 停止所有子进程
> quit
```

## 分步手动启动

### 终端 1：SLAM + 导航

```bash
sudo ~/.stop_ros.sh
rm -f /home/ubuntu/.ros/rtabmap.db
ros2 launch navigation rtabmap_navigation.launch.py localization:=false
ros2 launch navigation rtabmap_navigation.launch.py localization:=true

# 雷达建图和保存
python3 tools/radar_mapping.py radar_map

# 手动发布地图（相机建图不能用这个，因为效果很差）
ros2 run nav2_map_server map_server --ros-args --param yaml_filename:=/home/ubuntu/.ros/maps/camera_map.yaml
ros2 lifecycle set /map_server configure # 配置节点
ros2 lifecycle set /map_server activate # 激活节点（关键步骤！否则不发布数据）

# 手动发布rtabmap地图（需要启动nav后）
ros2 service call /rtabmap/publish_map rtabmap_msgs/srv/PublishMap "{global_map: true, optimized: true, graph_only: false}"
```

### 终端 2：RViz

```bash
ros2 launch navigation rviz_rtabmap_navigation.launch.py

# 监控覆盖率（仅依赖 SLAM 位姿）
ros2 launch coverage_evaluator coverage_evaluator.launch.py
```

### 终端 3：按需运行

> 修改 ROS2 包后需重新编译：`cd ros2_ws/ && colcon build --packages-select 包名 && source install/local_setup.sh`
> 常用`cd ros2_ws/ && colcon build --packages-select navigation path_coverage slam && source install/local_setup.sh`

#### 正常作业：
**A. 执行覆盖路径规划：**

```bash
ros2 launch path_coverage path_coverage.launch.py
# 在 RViz 中用 Publish Point 点击圈选区域，自动开始覆盖
```

#### 实验测试：



**D. 三组消融对照实验 (Group A/B/C)：**

```bash
python3 tools/test_coverage_comparison.py --mode a       # Group A: LiDAR + 原始
python3 tools/test_coverage_comparison.py --mode b       # Group B: RTAB-Map + 原始 (消融)
python3 tools/test_coverage_comparison.py --mode c       # Group C: RTAB-Map + 改进 (创新)
```
三组对比论证：A vs B 证明仅换传感器不够，B vs C 证明算法改进是必要条件，A vs C 证明完整方案可达传统水平。


**F. 监控覆盖率（外置相机记录）：**

见D:\python\Python\割草机导航\相机处理\README.md

**E. 运行标准测试：**

```bash
python3 tools/test1_slam_nav_test.py --mode rpe     # 闭合路径 RPE
python3 tools/test1_slam_nav_test.py --mode static  # 静态定位稳定性

python3 tools/test2_nav_cte_and_obstacle_test.py --mode cte  # 直线 CTE
python3 tools/test2_nav_cte_and_obstacle_test.py --mode obstacle --path 1to3    # 避障 1→3
python3 tools/test2_nav_cte_and_obstacle_test.py --mode obstacle --path 4to2    # 避障 4→2
```