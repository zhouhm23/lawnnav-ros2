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

**A. 执行覆盖路径规划：**

```bash
ros2 launch path_coverage path_coverage.launch.py
# 在 RViz 中用 Publish Point 点击圈选区域，自动开始覆盖
```

**B. 监控覆盖率（仅依赖 SLAM 位姿）：**

```bash
ros2 launch coverage_evaluator coverage_evaluator.launch.py
# 在 RViz 中用 Publish Point 点击 ≥3 个点圈选区域（末点靠近首点自动闭合）
# 终端会每秒打印覆盖率日志
# 查看实时覆盖率 topic: ros2 topic echo /coverage_ratio
# 切换区域：ros2 service call /reset std_srvs/srv/Empty
```

**C. 运行标准测试：**

```bash
# test1: SLAM 综合测试
python3 tools/test1_slam_nav_test.py --mode rpe     # 闭合路径 RPE
python3 tools/test1_slam_nav_test.py --mode static  # 静态定位稳定性
python3 tools/test1_slam_nav_test.py --mode all     # 全部

# test2: 导航控制测试
python3 tools/test2_nav_cte_and_obstacle_test.py --mode cte                     # 直线 CTE
python3 tools/test2_nav_cte_and_obstacle_test.py --mode obstacle --path 1to3    # 避障 1→3
python3 tools/test2_nav_cte_and_obstacle_test.py --mode obstacle --path 4to2    # 避障 4→2
```