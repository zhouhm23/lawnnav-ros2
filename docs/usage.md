# 使用方法

## 一键启动

```bash
python3 tools/start_path_coverage.py         # 默认
python3 tools/start_path_coverage.py --quiet # 安静模式
bash tools/start_path_coverage.sh            # Shell 兼容包装
```

---

## 分步手动启动

### 终端 1：SLAM + 导航

```bash
~/.stop_ros.sh
rm -f /home/ubuntu/.ros/rtabmap.db
ros2 launch navigation rtabmap_navigation.launch.py localization:=false
```

### 终端 2：RViz

```bash
ros2 launch navigation rviz_rtabmap_navigation.launch.py
```

### 终端 3：按需运行

> 修改 ROS2 包后需重新编译：`cd ros2_ws/ && colcon build --packages-select 包名 && source install/local_setup.sh`

**A. 执行覆盖路径规划：**

```bash
ros2 launch path_coverage path_coverage.launch.py
# 在 RViz 中用 Publish Point 点击圈选区域，自动开始覆盖
```

**B. 自动化覆盖测试：**

```bash
python3 tools/run_auto_coverage_test.py --mode mapping   # 建图 + 覆盖
python3 tools/run_auto_coverage_test.py --mode coverage  # 仅覆盖（需已有地图）
```

**C. 监控覆盖率（仅依赖 SLAM 位姿）：**

```bash
ros2 launch coverage_evaluator coverage_evaluator.launch.py
```

**D. 离线视频覆盖分析（车上 / Windows 均可运行）：**

```bash
# 车上 (ROS2 环境):
ros2 run coverage_evaluator run_camera_coverage --video test.mp4 --mask mask.png --visualize

# Windows: 把 camera_coverage.py 和 run_camera_coverage.py 放同一目录，然后:
pip install opencv-python matplotlib numpy
python run_camera_coverage.py --video test.mp4 --mask mask.png --visualize
```

**E. 运行标准测试：**

```bash
python3 tools/test1_slam_nav_test.py --mode rpe     # 闭合路径 RPE
python3 tools/test1_slam_nav_test.py --mode static  # 静态定位稳定性

python3 tools/test2_nav_cte_and_obstacle_test.py --mode cte  # 直线 CTE
python3 tools/test2_nav_cte_and_obstacle_test.py --mode obstacle --path 1to3    # 避障 1→3
python3 tools/test2_nav_cte_and_obstacle_test.py --mode obstacle --path 4to2    # 避障 4→2
```