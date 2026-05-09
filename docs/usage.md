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

> save test_map            ← 保存地图副本
> load test_map            ← 切换地图
> list                     ← 查看所有地图和区域

> region my_area           ← 在 RViz 中 Publish Point 圈选新区域并保存
> live test_180x240        ← ⭐ 建图模式下直接覆盖 (不切换定位，最稳定)
> coverage test_map test_180x240  ← 纯定位覆盖 (需 localization 稳定时)

> status                   ← 查看进程状态
> log [进程名]             ← 查看进程日志
> stop                     ← 停止所有子进程
> quit
```

> **区域管理**: 内置了 `test_180x240` 区域。用 `region <名称>` 可在 RViz 中圈选新的覆盖区域，保存到 `~/.ros/regions/`。

> **提示**: 旧版 `tools/start_path_coverage.sh` 仍可用，内部自动转发到新启动器。

---

## 分步手动启动

### 终端 1：SLAM + 导航

```bash
sudo ~/.stop_ros.sh
rm -f /home/ubuntu/.ros/rtabmap.db
ros2 launch navigation rtabmap_navigation.launch.py localization:=false
```

### 终端 2：RViz

```bash
ros2 launch navigation rviz_rtabmap_navigation.launch.py
```

### 终端 3：按需运行

> 修改 ROS2 包后需重新编译：`cd ros2_ws/ && colcon build --packages-select 包名 && source install/local_setup.sh`

#### 正常作业：
**A. 执行覆盖路径规划：**

```bash
ros2 launch path_coverage path_coverage.launch.py
# 在 RViz 中用 Publish Point 点击圈选区域，自动开始覆盖
```

#### 实验测试：
**B. 标准化覆盖测试：**

内置了 `test_180x240` 区域文件，无需手动圈选：

```bash
python3 launcher/start.py
# 1. 在 launcher 中建图 (手动驾驶)
> mapping
# 2. 保存为测试地图
> save test_map
# 3. 此后每次测试直接一键执行
> live test_180x240        ← ⭐ 建图模式下直接覆盖 (不切换定位，最稳定)
> coverage test_map test_180x240
```

**C. 监控覆盖率（仅依赖 SLAM 位姿）：**

```bash
ros2 launch coverage_evaluator coverage_evaluator.launch.py
```

**D. 离线视频覆盖分析（源码在Windows上，车里的已经过时）：**

见D:\python\Python\割草机导航\相机处理\README.md

**E. 运行标准测试：**

```bash
python3 tools/test1_slam_nav_test.py --mode rpe     # 闭合路径 RPE
python3 tools/test1_slam_nav_test.py --mode static  # 静态定位稳定性

python3 tools/test2_nav_cte_and_obstacle_test.py --mode cte  # 直线 CTE
python3 tools/test2_nav_cte_and_obstacle_test.py --mode obstacle --path 1to3    # 避障 1→3
python3 tools/test2_nav_cte_and_obstacle_test.py --mode obstacle --path 4to2    # 避障 4→2
```