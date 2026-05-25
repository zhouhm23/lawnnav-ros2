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
> region my_area           ← 在 RViz 中 Publish Point 圈选新区域并保存
> load test_map            ← 切换地图
> list                     ← 查看所有地图和区域

> coverage camera_map test_180x240   ← 纯定位覆盖
> live test_180x240        ← 建图模式下直接覆盖

> log [进程名]             ← 查看进程日志
> status                   ← 查看进程状态
> stop                     ← 停止所有子进程
> quit
```

## 分步手动启动

### 终端 1：

```bash
sudo ~/.stop_ros.sh
rm -f /home/ubuntu/.ros/rtabmap.db
# (a) 单相机
ros2 launch navigation rtabmap_camera_nav.launch.py                    # 建图 (localization:=false)
ros2 launch navigation rtabmap_camera_nav.launch.py localization:=true # 导航

# (b) 单雷达
ros2 launch slam slam_toolbox_lidar_slam.launch.py
ros2 launch navigation slam_toolbox_lidar_nav.launch.py

# (c) 视觉+雷达
ros2 launch navigation rtabmap_vslam_nav.launch.py                    # 建图 (localization:=false)
ros2 launch navigation rtabmap_vslam_nav.launch.py localization:=true # 导航

# 手动发布地图（相机建图不能用这个，因为效果很差）
ros2 run nav2_map_server map_server --ros-args --param yaml_filename:=/home/ubuntu/.ros/maps/camera_map.yaml
ros2 lifecycle set /map_server configure # 配置节点
ros2 lifecycle set /map_server activate # 激活节点（关键步骤！否则不发布数据）

# 手动发布rtabmap地图（需要启动nav后）
ros2 service call /rtabmap/publish_map rtabmap_msgs/srv/PublishMap "{global_map: true, optimized: true, graph_only: false}"
```

### 终端 2：

```bash
ros2 launch navigation rviz_rtabmap_navigation.launch.py # rviz可视化，建议在pc上运行


ros2 launch coverage_evaluator coverage_evaluator.launch.py # 监控覆盖率（仅依赖 SLAM 位姿）
```

### 终端 3：
```bash
cd ros2_ws/ && colcon build --packages-select 包名 && source install/local_setup.sh # 修改包后要重新编译
cd ros2_ws/ && colcon build --packages-select navigation path_coverage slam && source install/local_setup.sh # 常用

ros2 launch path_coverage path_coverage.launch.py # 在 RViz 中用 Publish Point 点击圈选区域，自动开始覆盖
```

#### 实验测试：

**阶段1：建图（人工操作，每种传感器建1次，共3张图）**

```bash
# ===== (a) 单相机建图 =====
python3 launcher/start.py
> mapping                    # 手动遥控遍历区域
> save camera_map            # 保存到 ~/.ros/maps/camera_map.db + .pgm/.yaml

# ===== (b) 单雷达建图 =====
# 终端1: 启动 SLAM
ros2 launch slam slam_toolbox_lidar_slam.launch.py
# 终端2: 遥控
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
# 终端3: 建图完成后保存 (先停止遥控，等几秒让地图稳定)
ros2 run nav2_map_server map_saver_cli -f /home/ubuntu/.ros/maps/radar_map
# 验证: ls ~/.ros/maps/radar_map.yaml ~/.ros/maps/radar_map.pgm

# ===== (c) 融合建图 =====
ros2 launch navigation rtabmap_vslam_nav.launch.py
# 手动遥控遍历区域后:
# 1. 保存 rtabmap 数据库
cp /home/ubuntu/.ros/rtabmap.db /home/ubuntu/.ros/maps/vslam_map.db
# 2. 发布并保存栅格地图
ros2 service call /rtabmap/publish_map rtabmap_msgs/srv/PublishMap "{global_map: true, optimized: true, graph_only: false}"
ros2 run nav2_map_server map_saver_cli -f /home/ubuntu/.ros/maps/vslam_map
# 验证: ls ~/.ros/maps/vslam_map.db ~/.ros/maps/vslam_map.yaml
```

**阶段2：SLAM 定位验证（3传感器 × 3次 = 9次）**

```bash
# 依次测三种传感器，各3次
python3 tools/test_slam_nav_test.py --sensor camera
python3 tools/test_slam_nav_test.py --sensor lidar
python3 tools/test_slam_nav_test.py --sensor vslam

# 流程: 导航1→2→3→4→1闭合矩形
#   - 每段自动测CTE
#   - 航点3,4,1暂停: 自动静态采集 + 手工输入地面真值
#   - 航点3→4段输入是否碰撞
# 原始数据: logs/pose/    论文数据: tools/results/slam_*.csv
```

**阶段3：全覆盖性能对照实验（最多6组 × 3次 = 18次）**

```bash
# 逐组跑:
python3 tools/test_coverage_comparison.py --sensor camera --algo ours
python3 tools/test_coverage_comparison.py --sensor camera --algo baseline
python3 tools/test_coverage_comparison.py --sensor lidar --algo baseline
python3 tools/test_coverage_comparison.py --sensor vslam --algo baseline
python3 tools/test_coverage_comparison.py --sensor lidar --algo ours
python3 tools/test_coverage_comparison.py --sensor vslam --algo ours

# 跑完生成插图
python3 tools/paper_figures.py ...

# 自动输出: tools/results/coverage_results.csv（覆盖率、耗时，仅正常完成时写入）
# evaluator 详细日志: logs/coverage/*_evaluator.log
# 外置相机视频分析（覆盖率/重复覆盖率/轨迹长度/碰撞）在 PC 端独立完成
```