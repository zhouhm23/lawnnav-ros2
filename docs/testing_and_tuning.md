# 测试历程与参数调优

## 一、测试脚本使用说明

进行测试前需启动基础环境（终端1: SLAM/导航，终端2: RViz），详见 [usage.md](./usage.md#终端分步手动启动与测试流程)。终端3 执行脚本。

> **坐标系**: 车起步点 = SLAM 原点 `(0,0)`，车头朝 +x，左侧朝 +y。yaw 不固定为 0，`normalize_angle(current - start)` 始终正确。

---

### 公共工具模块 `tools/test_utils.py`

| 组件 | 用途 |
|------|------|
| `yaw_from_quaternion` / `quaternion_from_yaw` / `normalize_angle` | 数学工具 |
| `make_pose_stamped` | 构造 PoseStamped |
| `CSVLogger` | 标准化 CSV，逐行 flush 防崩溃丢数据 |
| `StuckDetector` | 位姿滑动窗口卡死检测（导航场景） |
| `CmdVelMonitor` | 监控 /cmd_vel 活跃状态 |
| `rotate_360` | 两阶段闭环旋转（见下文） |
| `wait_for_localization` | 阻塞等待定位就绪 |

---

### 碰撞急停

所有导航脚本 (`goal1~4`, `goal_all`, `test2`) 内置：

```
StuckDetector: 5s 内位移 < 5cm 且角度 < 0.05rad
     AND
CmdVelMonitor: 1s 内有非零 /cmd_vel
     ↓
"COLLISION STUCK" -> cancel_goal -> skip
```

单 goal 超时 120s 兜底。

---

### 旋转方案演进 (test2 / test3)

| 版本 | 方案 | 问题 | 结论 |
|------|------|------|------|
| v1 | 固定 12.5s 开环 (2pi/0.5) | 车速 != 0.5 rad/s -> 转不满或过头 | 不靠谱 |
| v2 | 累积 delta | spin频率 > 定位频率时 delta=0 堆积 -> StuckDetector 误触发 | 不靠谱 |
| v3 | 绝对目标 yaw (normalize(start+2pi)) | 全周 target==start -> error=0 -> 0.1s "完成" | 不靠谱 |
| v4 | 双重条件 (累计>=351度 + yaw回起点) | 复杂，累计追踪漂移 | 不靠谱 |
| v5 | 定时 (2x余量) + 事后验证 | 车速快时 25s -> 一圈半 | 不靠谱 |
| **v6** | **两阶段: yaw_diff 先过 90度 再降回 15度 即停** | **精准 +/-15度** | **通过** |

**v6 原理**: `|normalize(current - start)|` 旋转时自然走 0->180->0 度。Phase 1 等 >90度 证明在转，Phase 2 等降回 <15度 即停。不依赖时间、不累积、不检测卡死。

---

### 地图数据源坑

| 尝试 | Topic | 结果 |
|------|-------|------|
| 1 | `/map` | RTAB-Map 模式下无 map_server -> 空 |
| 2 | `/rtabmap/grid_map` | RTAB-Map 默认 topic，但 launch 未 remap -> 裸名是 `/grid_map` |
| **3** | `/global_costmap/costmap` | Nav2 实时融合 RTAB-Map 点云，和 path_coverage 同源 |

test3 同时订阅 5 个 topic 自动选最佳源。计数跟随 path_coverage: `-1`=未知, `0~70`=自由, 其余=障碍。

---

### 1. goal1~4 / goal_all -- 固定路径测试

基础坐标，带碰撞急停 + 120s 超时。goal_all 完成后 60s 高频位姿采样输出 CSV。

### 2. test2 -- 轨迹误差测量

360度预旋转 -> 连续 4 目标 -> 1Hz 理论/实际位姿误差 -> CSV。带碰撞急停。

### 3. test3 -- 建图质量量化 (已通过)

| 项目 | 说明 |
|------|------|
| ROI | x in [0, 1.0], y in [-1.8, 0.0] |
| 流程 | 两阶段 360度 旋转 -> 自动选 map 源 -> 1Hz 采样 -> 连续5次值不变自动停止 |
| 输出 | `tools/map_quality_<timestamp>.csv` |
| 截图 | `tools/pic/test3_4_29_21_03.png` |

`_count()` 使用 path_coverage 同款 `floor/ceil` 栅格索引 + `costmap_max_non_lethal=70` 阈值。

---

## 二、建图参数调优

房间 15m x 10m，最终参数：

| 文件 | 参数 | 值 | 说明 |
|------|------|----|------|
| `rtabmap_params.yaml` + `rtabmap.launch.py` | `Grid/RangeMax` | **10.0** | 覆盖最远墙壁 |
| `nav2_params.yaml` | `obstacle_max_range` (x2) | **10.0** | costmap 障碍检测 |
| `nav2_params.yaml` | `raytrace_max_range` (x2) | **12.0** | 射线清理距离 |
| `nav2_params.yaml` | `planner.allow_unknown` | **true** | 允许穿越未知区 |

| 日期 | 问题 | 改动 | 效果 |
|------|------|------|------|
| 2026.04.29 | RViz 正前扇形灰色盲区 | `Grid/MinGroundHeight`: 0.20->-0.20, `GroundIsObstacle`: true->false | 已解决 |
| 2026.04.29 | 墙壁太远地板变未知 | range 系列 2.5->10.0 | 已解决 |

---

## 三、踩坑记录

1. **旋转检测不能用累计 delta** -- spin频率 > 定位频率时重复读同值，delta=0 填充 StuckDetector 误判。
2. **全周旋转不能用绝对目标 yaw** -- `normalize(start+2pi) == start`，起点即终点。
3. **RTAB-Map 下 /map 无人发布** -- localization.launch 不启动，map_server 不运行。正确源是 `/global_costmap/costmap`。
4. **RTAB-Map grid_map 裸名是 /grid_map** -- launch 里只 remap 了 cloud_map。
5. **spin_once != 定时器** -- 回调处理时间不定，不适合精确时序。两阶段 yaw 检测规避此问题。
6. **Nav2 costmap 值域** -- `-1`=未知, `0~70`=可通行，非标准 `0=free, 100=occ`。须用 path_coverage 同款阈值。
7. **CSVLogger 每行 flush** -- 否则 Ctrl+C 时缓冲区内数据丢失。
