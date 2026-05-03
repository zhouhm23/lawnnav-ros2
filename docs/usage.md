# 使用方法与操作指南

## 一键启动方式

推荐使用脚本一键启动系统（注意：目前不包括启动路径覆盖本身）：

```bash
python3 tools/start_path_coverage.py
```

**其他启动选项：**
- **安静模式**（精简终端输出，仅保留警告和错误）：
  ```bash
  python3 tools/start_path_coverage.py --quiet
  ```
- **兼容旧指令**（内部会转换为 Python 启动器）：
  ```bash
  bash tools/start_path_coverage.sh
  ```

---

## 终端分步手动启动与测试流程

实际工作与调试测试中，需要多个终端相互配合。**终端 1** 和 **终端 2** 是整个系统的**基础启动环境**，任何测试都必须依赖它们先行运行。**终端 3** 及其他终端用于按需运行具体的测试脚本。

### 👉 终端 1：基础启动环境（SLAM与导航栈）
此终端主要负责清理历史状态，并启动核心建图与导航进程。
```bash
# 1. 关闭手机控制 APP，节约系统性能
~/.stop_ros.sh

# 2. 删除之前的地图缓存数据（确保每次启动是全新的建图环境）
rm -f /home/ubuntu/.ros/rtabmap.db

# 3. 启动视觉导航与 SLAM（非纯定位模式）
ros2 launch navigation rtabmap_navigation.launch.py localization:=false
```

### 👉 终端 2：基础启动环境（RViz 可视化界面）
此终端用于启动用户界面，以从视觉上监控建图效果和导航状态。
*(请在终端 1 的程序完全拉起后再启动)*
```bash
ros2 launch navigation rviz_rtabmap_navigation.launch.py
```

### 👉 终端 3：执行测试与功能脚本
在终端 1 和 2 共同构成稳定的基础环境后，可在终端 3 运行不同的功能模块。

**情景 A：执行全区域路径覆盖规划**
```bash
ros2 launch path_coverage path_coverage.launch.py
```

**情景 B：执行建图空地测算量化测试 (`test3`)**
```bash
python3 tools/test3_measure_map_quality.py
```

**情景 C：执行运动路径误差测量测试 (`test2`)**
```bash
python3 tools/test2_measure_trajectory_error.py
```

## 
```bash
python3 tools/test4_measure_map_accuracy.py
python3 -c "
import pandas as pd
df = pd.read_csv('tools/obstacle_dimension_accuracy.csv')
print(df[['run_id','err_l_m','err_w_m','err_h_m','mean_vertex_err']].describe())
"
```