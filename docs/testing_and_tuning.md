
# 测试历程与参数调优

## 一、测试脚本使用说明

进行测试前需启动基础环境（终端1: SLAM/导航，终端2: RViz），详见 [usage.md](./usage.md#终端分步手动启动与测试流程)。终端3 执行脚本。

> **车本征坐标系**: 车起步点 = SLAM 原点 `(0,0)`，车头朝 +x，左侧朝 +y。yaw 不固定为 0，`normalize_angle(current - start)` 始终正确。
> **论文坐标系**：以车初始朝向为y+,右边为x+，车初始位置为1(0.4,0),；其他点为2(0.4,1.8),3(1.4,1.8),4(1.4,0)，障碍物左下角(0.9,1.2)，右上角(0.9+0.26,1.2+0.18)

---

### 公共工具模块 `tools/test_utils.py`

| 组件 | 用途 |
|------|------|
| `yaw_from_quaternion` / `quaternion_from_yaw` / `normalize_angle` | 数学工具 |
| `make_pose_stamped` | 构造 PoseStamped |
| `CSVLogger` | 标准化 CSV，逐行 flush 防崩溃丢数据 |
| `AppendingCSVLogger` | 固定文件名 CSV，追加模式，自动递增 run_id |
| `StuckDetector` | 位姿滑动窗口卡死检测（导航场景） |
| `CmdVelMonitor` | 监控 /cmd_vel 活跃状态 |
| `rotate_360` | 两阶段闭环旋转（见下文） |
| `rotate_by_angle` | 闭环旋转指定角度（累积 delta 法） |
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

### 1. test1_slam_nav_test -- SLAM与导航综合测试 ✅

**统一测试入口**，替代旧版 `goal1-4.py`、`goal_all.py`、`test2`、`test3`。

```bash
python3 test1_slam_nav_test.py --mode rpe     # 仅闭合路径RPE
python3 test1_slam_nav_test.py --mode static  # 仅静态定位稳定性
python3 test1_slam_nav_test.py --mode all     # 全部（默认）
```

**RPE 模式流程**：
1. 按 2→3→4→1 顺序导航闭合矩形路径（带 120s 超时 + 定位丢失检测）
2. 每个目标点到达后记录 SLAM 位姿，等待用户终端输入论文坐标系地面真值 `x y yaw_deg`
3. 脚本自动计算逐段 RPE（4段）和端到端闭合误差
4. 结果写入 `tools/rpe_results.csv`

**Static 模式流程**：
1. 以 5Hz 采样位姿 60s → `tools/pose_log_*.csv`
2. 以第一帧为参考，计算 RMSE 抖动 + 最大漂移量
3. 结果写入 `tools/static_stability_results.csv`

> **坐标系**: 用户以论文坐标系输入 GT（y⁺=车头, x⁺=右侧, 1=(0.4,0)），脚本自动转换为 Nav2 map 坐标系。RPE 用相对位移差，不受全局坐标不对齐影响。
>
> **已废弃**: goal1~4.py、goal_all.py、test2/3_*.py（文件保留但标注 DEPRECATED）。

#### 实测结果 (2026.05.04)

**（1）闭合路径相对位姿误差 — 5 组实验**

| 段 | Run 1 | Run 2 | Run 3 | Run 4 | Run 5 | Mean ± Std |
|---|:-----:|:-----:|:-----:|:-----:|:-----:|:---:|
| 2→3 | 0.054 | 0.057 | 0.076 | 0.088 | 0.108 | 0.077 ± 0.022 |
| 3→4 | 0.046 | 0.077 | 0.047 | 0.065 | 0.096 | 0.066 ± 0.021 |
| 4→1 | 0.060 | 0.074 | 0.077 | 0.085 | 0.094 | 0.078 ± 0.013 |
| 1→2 | 0.084 | 0.084 | 0.054 | 0.049 | 0.073 | 0.069 ± 0.016 |

> 整体逐段位置 RPE **0.072 ± 0.019 m**（N=20）。端到端闭合误差 **0.069 ± 0.016 m**。
> 闭合路径总周长约 5.6 m，闭合误差占比约 1.2%，各段误差均匀未出现累积漂移趋势。
> 航向 RPE 因人工 GT 估读精度有限（±30°），不作为主要评指标。

**（2）静态定位稳定性 — 3 组实验**

| Run | RMSE 位置 (μm) | RMSE 航向 (°) | 最大位置漂移 (μm) | 最大航向漂移 (°) |
|:---:|:---:|:---:|:---:|:---:|
| 1 | <1 | 0.012 | <1 | 0.036 |
| 2 | <1 | 0.026 | <1 | 0.064 |
| 3 | <1 | 0.014 | <1 | 0.039 |

> 60s 静止采样（5Hz）。位置漂移 < 1 μm（可忽略），航向 RMSE < 0.03°，无显著积分漂移。
> IMU 零偏被融合滤波器有效抑制。

**参数调整 (2026.05.04)**：Nav2 目标容差收紧 `xy_goal_tolerance: 0.1→0.05 m`，`yaw_goal_tolerance: 0.1→0.05 rad`，`rotate_to_heading_min_angle: 0.1→0.05 rad`，预期逐段 RPE 从 0.08→0.05 m 区间。

---

### 2. test2 — 导航控制与避障指标测试 ✅

**评测对象**: Nav2 导航模块在已知地图与定位输入条件下的直线跟踪能力与绕障安全性。

```bash
python3 tools/test2_nav_cte_and_obstacle_test.py --mode cte                    # 仅直线跟踪CTE
python3 tools/test2_nav_cte_and_obstacle_test.py --mode obstacle --path 1to3   # 仅避障 1→3
python3 tools/test2_nav_cte_and_obstacle_test.py --mode obstacle --path 4to2   # 仅避障 4→2
python3 tools/test2_nav_cte_and_obstacle_test.py --mode all                    # 全部（默认，仅CTE）
```

> 避障模式每次只测一条路径，测完一条后 Ctrl+C 重新启动脚本指定另一条 `--path`，避免累计里程计漂移。

**测试路径**（map 坐标系，x⁺=车头，y⁺=左侧）：

| 路径 | 起点 | 终点 | 类型 |
|------|------|------|------|
| 1→2 | (0, 0) | (1.8, 0) | CTE 闭合矩形第1段，水平 y≡0，1.8m |
| 2→3 | (1.8, 0) | (1.8, -1.0) | CTE 闭合矩形第2段，竖直 x≡1.8，1.0m |
| 3→4 | (1.8, -1.0) | (0, -1.0) | CTE 闭合矩形第3段，水平 y≡-1.0，1.8m |
| 4→1 | (0, -1.0) | (0, 0) | CTE 闭合矩形第4段，竖直 x≡0，1.0m |
| 1→3 | (0, 0) | (1.8, -1.0) | 对角线绕障（障碍物箱 ~(1.2, -0.5)） |
| 4→2 | (0, -1.0) | (1.8, 0) | 对角线绕障 |

> 障碍物箱：左下角地图坐标 ~(1.11, -0.58)，尺寸 0.18×0.26m。（对应论文坐标：左下角 (0.9, 1.2)，长 0.26m × 宽 0.18m，转换后 x_map=y_paper, y_map=-x_paper+0.4）

---

#### CTE 模式 — 闭合矩形路径跟踪横向误差

| 项目 | 说明 |
|------|------|
| 流程 | 先导航到起点 1，再沿闭合矩形 1→2→3→4→1 逐段导航，每段以 1Hz 采样 SLAM 位姿 |
| 轨迹集 | 每段独立 P_tr = {p_0, p_1, ..., p_K}，每个轨迹点含 (t, x, y, yaw) |
| 期望路径 | 每段为轴对齐直线：1→2: y≡0; 2→3: x≡1.8; 3→4: y≡-1.0; 4→1: x≡0 |
| 横向误差 | 水平段: e_cte,k = \|p_y - y_ref\|；竖直段: e_cte,k = \|p_x - x_ref\| |
| 输出 | `tools/cte_results.csv`（逐段+整体汇总）+ `tools/trajectory_cte_{label}_{ts}.csv`（段原始轨迹） |

**CTE 指标公式**：

$$e_{cte,RMSE} = \sqrt{\frac{1}{K+1}\sum_{k=0}^{K} (e_{cte,k})^2}$$

$$e_{cte,MAX} = \max\{e_{cte,k}\}$$

**CTE 结果记录**：

| Run | 1→2 RMSE | Max | 2→3 RMSE | Max | 3→4 RMSE | Max | 4→1 RMSE | Max | 整体 RMSE | 整体 Max | 备注 |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---|
| — | — | — | — | — | — | — | — | — | — | — | 待测试 |

> 采样频率 1Hz。CTE 覆盖水平/竖直双向直线段，评估 PurePursuit 控制器在往复式覆盖路径中的综合跟踪精度。

---

#### 避障模式 — 安全裕度与目标到达质量

| 项目 | 说明 |
|------|------|
| 流程 | 每次只测一条路径（1→3 或 4→2，通过 --path 指定），重复 3 遍；需测另一条时重启脚本 |
| 碰撞记录 | 到达目标后人工终端输入 collision (0/1) |
| 最小间隙 | 由 SLAM 轨迹 + 障碍物几何 + 车体包络 (0.215×0.18m) 自动计算 d_min |
| 目标到达精度 | 自动计算 SLAM 稳态位姿与目标位姿的位置残差 / 航向残差 |
| 输出 | `tools/obstacle_avoidance_results.csv` + `tools/trajectory_obs_*.csv` |

**d_min 自动计算**: 对每条绕障路径以 5Hz 采样 SLAM 轨迹，逐点将车体包络矩形 4 角变换到世界坐标，计算各角点到障碍物矩形的最短距离，取全程最小值。

**车体包络矩形**: 长 0.215m（沿车头 +x）× 宽 0.18m（沿车体左侧 +y），半长 0.1075m，半宽 0.09m。

**目标到达精度**（Nav2 判定到达且车体静止后）：

$$e_{pos} = \sqrt{(x_{slam} - x_{goal})^2 + (y_{slam} - y_{goal})^2}$$

$$e_{yaw} = |\operatorname{normalize}(\psi_{slam} - \psi_{goal})|$$

**数据筛选**: collision 与 d_min 须逻辑一致才视为有效——collision=1 时 d_min 应 ≈0（车体已接触障碍物），collision=0 时 d_min 应 >0（有安全间隙）。冲突条目（collision=1 但 d_min>0，或 collision=0 但 d_min=0）予以剔除。冲突原因主要是 1 Hz SLAM 轨迹未能捕获最接近时刻、或 map 坐标系与实物有厘米级偏移。此外前 6 组（Run 1~6）d_min 为人工手记，精度不可靠，一并排除。

**避障结果记录（有效数据，共 9 组）**：

| Run | 路径 | collision | d_min (mm) | pos_err (m) | slam_x | slam_y | 备注 |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---|
| 8 | 1→3 | 0 | 83.7 | 0.0474 | 1.753 | −0.996 | |
| 9 | 1→3 | 0 | 78.5 | 0.0472 | 1.753 | −1.002 | |
| 10 | 1→3 | **1** | **0.0** | 0.0479 | 1.769 | −0.964 | 碰撞 ✓ |
| 13 | 4→2 | 0 | 106.8 | 0.0480 | 1.759 | −0.024 | |
| 14 | 4→2 | 0 | 119.7 | 0.0449 | 1.762 | −0.024 | |
| 15 | 4→2 | 0 | 129.6 | 0.0453 | 1.761 | −0.023 | |
| 18 | 1→3 | 0 | 193.6 | 0.0468 | 1.755 | −1.013 | |
| 20 | 1→3 | 0 | 21.4 | 0.0467 | 1.773 | −0.962 | |
| 21 | 1→3 | 0 | 154.4 | 0.0484 | 1.754 | −1.016 | |

**剔除条目**（共 12 组）：

| Run | 原因 |
|:---:|:---|
| 1~6 | d_min 人工手记，精度不可靠 |
| 7 | collision=1 但 d_min=28.6（冲突） |
| 11, 12, 16, 19 | collision=0 但 d_min=0（SLAM 漂移致假穿透） |
| 17 | collision=1 但 d_min=40.0（冲突） |

**汇总统计**：

| 指标 | 1→3 (5+1 碰撞) | 4→2 (3 组) |
|------|:---:|:---:|
| 碰撞率 | **16.7%** (1/6) | **0%** (0/3) |
| d_min（无碰撞） | **106.3 mm** (21~194) | **118.7 mm** (107~130) |
| 目标位置残差 | **0.0474 ± 0.0006 m** | **0.0461 ± 0.0016 m** |

> 1→3 方向 d_min 波动大（21~194 mm），反映对角线穿越障碍物区域时局部路径选择的随机性（Nav2 局部规划器每次可能选择不同绕行侧）。4→2 方向 d_min 稳定在 ~12 cm，安全裕度良好。目标位置残差全部在 Nav2 容差 0.05 m 以内，绕行扰动后控制器可稳定恢复终端精度。
>
> 容差沿用 Nav2 当前值：`xy_goal_tolerance=0.05m`, `yaw_goal_tolerance=0.05rad`。
> Ctrl+C 中断时会自动 cancel goal，车安全停止。

### 3. test3 -- 建图质量量化 (已通过)

| 项目 | 说明 |
|------|------|
| ROI | x in [0, 1.0], y in [-1.8, 0.0] |
| 流程 | 两阶段 360度 旋转 -> 自动选 map 源 -> 1Hz 采样 -> 连续5次值不变自动停止 |
| 输出 | `tools/map_quality_<timestamp>.csv` |
| 截图 | `tools/pic/test3_4_29_21_03.png` |

`_count()` 使用 path_coverage 同款 `floor/ceil` 栅格索引 + `costmap_max_non_lethal=70` 阈值。

### 4. test4 -- 地图尺寸精度与一致性评估 (✅ 已通过)

**目的**: 从物理尺寸还原度与结构连续性两个维度，对 SLAM 建图结果进行复合评估。

**实物布置**: 规则六面体箱子，左下角 `(0.5, 1.2, 0)`，对角 `(0.76, 1.38, 0.39)`。
尺寸: 长 0.26m × 宽 0.18m × 高 0.39m，体积 0.018252m³。

**数据源**:

| 维度 | 数据源 | Topic |
|------|--------|-------|
| 2D 自动 | Nav2 全局代价地图 | `/global_costmap/costmap` (仅参考) |
| 3D 自动 | RTAB-Map 累计点云 | `/rtabmap/cloud_map` (TRANSIENT_LOCAL, 百分位过滤) |
| **2D 手动** | **RViz 人工数格子** | **cell=0.05m (Grid/CellSize)** |

**流程**:
1. 360° 预旋转 → 导航 4 目标矩形路径（同 test2）
2. 回到原点后等待地图稳定（连续 5 次不变或最长 30s）
3. 自动提取 2D/3D 包络盒 → 自动写入 CSV
4. **在 RViz 上手数障碍物格子数 → 事后填入 CSV 的 `manual_*` 列**

> ⚠️ **严禁中途旋转**：在航点间旋转（270°）会导致障碍物出现多重叠影，严重破坏建图质量。

**输出**: `tools/obstacle_dimension_accuracy.csv`（三组独立列 + manual 空列待填）

**CSV 三组列**:

| 组 | 前缀 | 内容 |
|----|------|------|
| 2D 自动 | `2d_auto_*` | 长宽、包络盒、误差、栅格数（仅参考） |
| 3D 自动 | `3d_auto_*` | 长宽高、体积、8顶点误差、百分位过滤标记 |
| 2D 手动 | `manual_*` | 栅格数 L/W、L/W(m)、误差 → **测试后人工填写** |
| 参考 | `actual_*` | 实际物理尺寸 |

---

#### 实测结果 (5.01-5.03, 19 次运行)

**迭代历程**:

| 日期 | 方法 | 核心问题 | 结论 |
|------|------|----------|------|
| 5.01 R1-3 | costmap `>70` 提取 | inflation 膨胀 +250%/+100% | 阈值 ≥100 |
| 5.01 R4-8 | `>=100`, ROI_MARGIN=0.39 | ROI 覆盖墙壁, bbox 被环境拉大 | ROI 紧贴箱子 |
| 5.01 R9-11 | 窄 ROI + inflation 减 0.20m | obstacle_layer 展宽无法数学消 | **放弃自动 2D** |
| **5.03 R12-13** | **点2 270° 旋转** | **重影破坏建图** | **禁止中途旋转** |
| **5.03 R14-19** | **360° 预转 + RViz 数格子** | **精度良好** | **✅ 最终方案** |

**最终人工测量 (2026.05.03, 5 有效组)**:

| 实验# | 栅格 (L×W) | L (m) | W (m) | err_l | err_w | err_l% | err_w% |
|:-----:|:----------:|:-----:|:-----:|:-----:|:-----:|:-----:|:-----:|
| 3 | 6×5 | 0.30 | 0.25 | +0.04 | +0.07 | +15% | +39% |
| 4 | 5×4 | 0.25 | 0.20 | −0.01 | +0.02 | −4% | +11% |
| 5 | 5×3 | 0.25 | 0.15 | −0.01 | −0.03 | −4% | −17% |
| 7 | 5×4 | 0.25 | 0.20 | −0.01 | +0.02 | −4% | +11% |
| 8 | 6×4 | 0.30 | 0.20 | +0.04 | +0.02 | +15% | +11% |
| **Mean±Std** | — | **0.27±0.03** | **0.20±0.04** | **+0.01±0.03** | **+0.02±0.04** | **+4±10%** | **+11±20%** |

> 实验#1-2: 重影严重 → 舍去。实验#6: 起点多转一圈 odom 累积误差过大 → 舍去。

---

#### 结论

**✅ 人工 2D 测量通过** — 5 组有效数据长宽误差均值 < 0.03m，满足精度要求。

**❌ 自动 2D 测量不可行** — Nav2 costmap obstacle_layer 固有展宽（传感器投影 + 0.05m 栅格量化 + 多视角融合）无法数学消除，即使排除 inflation 层仍有每侧 0.10-0.15m 顽固膨胀。

**⚠️ 中途旋转破坏建图** — 航点间旋转导致 RTAB-Map 产生重影伪影，禁用。

**⚠️ odom 累积需控制** — 起点多转一圈使里程计误差累积不可用；每条测试严格控制总旋转量（≤360° 预转）。

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

0. **git**：不能在~下git提交（这里似乎有其他.git），要在~/ros2_ws/src下git提交
1. **旋转检测不能用累计 delta** -- spin频率 > 定位频率时重复读同值，delta=0 填充 StuckDetector 误判。
2. **全周旋转不能用绝对目标 yaw** -- `normalize(start+2pi) == start`，起点即终点。
3. **RTAB-Map 下 /map 无人发布** -- localization.launch 不启动，map_server 不运行。正确源是 `/global_costmap/costmap`。
4. **RTAB-Map grid_map 裸名是 /grid_map** -- launch 里只 remap 了 cloud_map。
5. **spin_once != 定时器** -- 回调处理时间不定，不适合精确时序。两阶段 yaw 检测规避此问题。
6. **Nav2 costmap 值域** -- `-1`=未知, `0~70`=可通行，非标准 `0=free, 100=occ`。须用 path_coverage 同款阈值。
7. **CSVLogger 每行 flush** -- 否则 Ctrl+C 时缓冲区内数据丢失。
8. **costmap `>70` 阈值不适用于障碍物尺寸测量** -- Nav2 costmap 值域中 71-99 为 inflation 膨胀区域，`>70` 判定会将膨胀层一起计入。纯尺寸测量应使用 `>=100` 或原始 RTAB-Map grid。
9. **3D 点云高度测量优于 2D costmap 尺寸** -- `/rtabmap/cloud_map` 点云提取的高度误差远优于 costmap 2D。对于需要 3D 尺寸的场景，点云是更可靠的数据源。
10. **ROI 边距过大会混入环境障碍物** — ROI_MARGIN 必须紧贴目标障碍物（±0.15m），否则附近墙壁/物体将被纳入包络盒。
11. **航点中途旋转导致重影** — 2026.05.03 test4 发现：在矩形路径航点间原地 270° 旋转会导致 RTAB-Map 产生多重叠影伪影，严重破坏建图质量。**中途禁止任何旋转**。
12. **odom 累积误差与总旋转量正相关** — 实验#6 因起点多转了一圈，里程计误差累积使定位漂移，最终数据不可用。控制每次测试的总旋转量 ≤360°。

## 四、覆盖率测试
### 第一次测试
问题与错误：
终端1：
[ekf_node-5] Failed to meet update rate! Took 0.011352473000000000078seconds. Try decreasing the rate, limiting sensor output frequency, or limiting the number of sensors.
[component_container_isolated-17] [ERROR] [1778148479.652405145] [controller_server]: Exception in transformPose: Lookup would require extrapolation into the future.  Requested time 1778148473.232299 but the latest data is at time 1778148471.499123, when looking up transform from frame [odom] to frame [map]
[rtabmap-14] [WARN] [1778148572.098544686] [rtabmap]: rtabmap: Did not receive data since 5 seconds! Make sure the input topics are published ("$ ros2 topic hz my_topic") and the timestamps in their header are set. If topics are coming from different computers, make sure the clocks of the computers are synchronized ("ntpdate"). If topics are not published at the same rate, you could increase "sync_queue_size" and/or "topic_queue_size" parameters (current=30 and 1 respectively).
[rtabmap-14] rtabmap subscribed to (approx sync):
[rtabmap-14]    /odom \
[rtabmap-14]    /rgbd_image

终端2：
[INFO] [1778148609.328599069] [auto_coverage_test]:   [1025s] 褰撳墠瑕嗙洊鐜? 0.0% (coverage_evaluator)

终端3：
[path_coverage_node.py-1] [WARN] [1778148529.818684948] [path_coverage]: Navigation timed out!
[path_coverage_node.py-1] [ERROR] [1778148963.291139211] [path_coverage]: Timed out waiting for goal to be accepted by the server.

### 第二次测试：localization 模式地图无白色区域

**现象**: 建图完成、切换 `localization:=true` 后重启，RViz 中 costmap 几乎全为障碍物（灰色/黑色），几乎无可通行白色区域，path_coverage 无路可走。

**根因**: `rtabmap.db` 只存点云特征（用于 RTAB-Map 视觉定位），**不包含 Nav2 可用的栅格地图 (pgm+yaml)**。localization 模式下 RTAB-Map 只做定位，没有 map_server 发布 `/map` 话题 → Nav2 的 `static_layer` 没有数据 → costmap 只剩 obstacle_layer 的点云障碍物（墙壁），无白色自由空间 → path_coverage 无路可走。

**修复方案** (2026.05.08):
覆盖模式启动时，额外启动 `map_server` 播放之前保存的栅格地图:

```bash
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=<path>.yaml
```

launcher 的 `save` 命令同时保存 rtabmap.db + grid map (pgm+yaml)，`coverage` 命令自动启动 map_server。

> **已废弃的错误方案**: 把 `/rtabmap/cloud_map` 改为 `/rtabmap/cloud_obstacles` — 这会导致建图阶段也丢失全部障碍物信息。

### 第三次测试：

1.
[OK] 鍦板浘宸叉仮澶? test_map
[INFO] 鍦板浘: test_map, 鍖哄煙: test_180x240.yaml
[INFO] === 鍚姩绾畾浣嶈鐩栨ā寮?(localization:=true) ===
[WARN] /home/ubuntu/ros2_ws/src/install/local_setup.sh not found, continuing without workspace setup
[INFO] 鍚姩 navigation
[INFO] 鍚姩 map_server 鍙戝竷鏍呮牸鍦板浘...
[WARN] /home/ubuntu/ros2_ws/src/install/local_setup.sh not found, continuing without workspace setup
[INFO] 鍚姩 map_server
[WARN] /home/ubuntu/ros2_ws/src/install/local_setup.sh not found, continuing without workspace setup
[INFO] 鍚姩 rviz
[WARN] /home/ubuntu/ros2_ws/src/install/local_setup.sh not found, continuing without workspace setup
[INFO] 鍚姩 path_coverage
[WARN] /home/ubuntu/ros2_ws/src/install/local_setup.sh not found, continuing without workspace setup
[INFO] 鍚姩 evaluator
[INFO] 鍙戝竷瑕嗙洊鍖哄煙...
鍙戝竷鍖哄煙 'test_180x240' (4 椤剁偣) 鍒?/clicked_point ...
  椤剁偣 1: (0.000, 0.400)
  椤剁偣 2: (2.400, 0.400)
  椤剁偣 3: (2.400, -1.400)
  椤剁偣 4: (0.000, -1.400)
  闂悎 鈫?澶氳竟褰㈠簲宸插畬鎴?瀹屾垚 鉁?[OK] 瑕嗙洊妯″紡灏辩华 鈥?鍖哄煙宸插彂甯冿紝寮€濮嬭鐩栦綔涓?

运行脚本没必要local_setup.sh，这个是编译后才需要的
2.
> mapping # 建图的时候，地图会突然出现小白点，然后膨胀成障碍物：我没有改任何导航代码，可能是相机有污渍
> save test_map
> coverage test_map test_180x240 # 明明我都启动mapping，结果还重新启动rviz，导致运行很慢；小车未开始覆盖，我看终端发现没启动path_coverage，而且栅格地图还是空白，不清楚为什么

### 第四次测试

**问题汇总**:

1. **map_server 不工作** — lifecycle 节点需手动 `configure → activate`，仅 spawn 不够
2. **grid map 保存为空** — `map_saver_cli` 默认订阅 `/map`，但 RTAB-Map 发布在 `/rtabmap/grid_map`，需 `-r /map:=/rtabmap/grid_map`
3. **launcher 时序过紧** — RPi 上 ROS2 发现慢，path_coverage 等节点需更长等待时间

**修复** (2026.05.09):

1. map_server 启动后执行 `ros2 lifecycle set /map_server configure && activate`
2. map_saver_cli 加重映射 `-r /map:=/rtabmap/grid_map`
3. launcher 时序改为匹配用户手动流程的验证时间线：
   - navigation → 2s → RViz → 5s → 等 30s → path_coverage + evaluator → 10s → 发布区域
4. 新增 `live` 命令 — mapping 模式下直接覆盖（不切换 localization），规避重定位不可靠问题
5. `path_coverage_params.yaml`: `drive_max_non_lethal` 和 `expand_max_non_lethal` 从 0 提升到 50，容忍 mapping 模式下的 inflation 灰色区域

### 第五次测试
这次测试中我5次启动覆盖时只有最后1次成功(live模式)，但车还是定位不准，有时候空的地方还是会变成黑色区域，有时候车还会碰到障碍物（膨胀层小）。而且最恶劣的是这次车在覆盖进度快一半时突然停止行走，rviz里显示突然红色边框消失，绿色路径消失。总之目前问题很多都是概率性的，我也不知道怎么办。建议去看logs/start_logs，不过不要直接读取完整文件（非常大），建议从最后的几行读起。问题的关键很可能在日志上。

### 第六次测试
以下均使用coverage test_map test_180x240
第1次启动时点1到点2没连上，所以启动失败，怀疑是点1没发布成功；第二次成功启动，但可能由于我手动重定位不好，导致与障碍物碰撞，但还是完成全覆盖任务，且pc上运行视频分析测得
```
录制时长:     866.6 s
总帧数:       22784
有效轨迹点:   9148
区域覆盖率:   90.4%
重复覆盖率:   90.0%
轨迹长度:     37.44 m
```
碰撞原因应该是重定位问题，因为我障碍物宽度和车宽差不多，重定位误差对碰撞的影响很大。可能需要实现自动重定位模式（起点任意），但也要保留刚成功的手动重定位模式（起点需手动放到map原点）。

### 第七次测试
能完成覆盖，但车后面卡住了，我看rviz里显示车不动，但现实明明在动，一直不停，最后我手动停止了。而且过程中还是碰撞，我看点云和黑色区域不符，车歪了都不自动修正位置。

**根因** (2026.05.09 事后分析):
EKF 传感器融合链中，RTAB-Map 视觉 SLAM 的位姿修正完全没有回灌给 EKF：
```
odom0: odom_raw     ← 轮式里程计 (可靠)
odom1: odom_rf2o    ← 激光里程计 (LD19 未连，死输入)
imu0:  imu          ← IMU 航向 (部分补偿)
```
轮式里程计在无外部绝对参考时必然漂移 → EKF 累积漂移 → Nav2 costmap 与实际不一致 → 碰撞。

**修复** (2026.05.09):
在 `driver/controller/config/ekf.yaml` 新增 `pose0: rtabmap/localization_pose`，使用 `differential: true` 将 RTAB-Map 的绝对位姿变化量转为速度修正注入 EKF：
```yaml
pose0: rtabmap/localization_pose
pose0_config: [true, true, false, false, false, true, ...]
pose0_differential: true
pose0_rejection_threshold: 3.0
```
原理：RTAB-Map 视觉特征匹配每秒检测位姿漂移 → EKF 将位姿差分转为速度修正 → 持续消除累积误差。不影响 world_frame 和 TF 树。
⚠️ 注意: `driver/` 不在 git 版本控制中，此文件修改不会被 git 追踪。需备份或手动记录。

现在要做对照的话就要想办法让二者共存，然后通过测试脚本分别进行实验

### 第八次测试
创新组完成完整覆盖任务，区域覆盖率86.7%为可接受数值，关键是无任何碰撞，说明定位精度提高。
对照组正常启动导航和覆盖任务，但由于覆盖脚本缺陷导致中途停止，这说明对照组选择的路径覆盖包版本不对，不是能基本完成的版本。

### 第九次改进 (2026.05.10)：三组消融实验设计 + 代码鲁棒性升级

**背景**: test 8 中对照组因 `get_closest_possible_goal` 返回 None 导致 `'NoneType' object has no attribute 'pose'` 崩溃，全程覆盖率仅 17.4%，对照实验无法得出有效结论。同时创新组在 test 5/7 中偶发中途停止（红色边框消失/路径消失），需要进一步增强监控和恢复能力。

**三组消融实验设计**:

| 组 | 传感器 | 算法 | 目的 |
|:---|:---|:---|:---|
| **A** (传统基准) | LiDAR | 原始 path_coverage | 传统方案基线 |
| **B** (消融组) | RTAB-Map 视觉 | 原始 path_coverage | 证明仅换传感器不够 |
| **C** (创新组) | RTAB-Map 视觉 | 改进 path_coverage | 你的完整方案 |

论证逻辑: A vs B（换传感器后崩溃）→ B vs C（加算法修复）→ A vs C（完整方案可达传统水平）。

**代码改动**:

1. **Baseline 修复** (`path_coverage_node_baseline.py`):
   - `get_closest_possible_goal` 返回前添加 `None` 守卫（与创新版一致）
   - `drive_path` 单个 waypoint 添加 try/except 守卫（失败跳过而非全崩溃）
   - None 返回时添加 warn 日志（便于追踪差异）
   - ⚠ 不添加 retry/costmap_wait/sleep(0.5) 等改进，精确量化算法贡献

2. **创新版增强** (`path_coverage_node.py`):
   - 添加 15s 心跳定时器 `_heartbeat_callback`（覆盖中打印 `[HEARTBEAT] node alive, state=...`）
   - `drive_path` 外层包裹进程级 try/except（崩溃时打印完整 traceback）
   - Cell 失败后自动恢复导航（回到 cell 质心，30s 超时，不阻塞）
   - 覆盖开始/结束时自动追踪 `_coverage_start_time` + `_cover_state`

3. **对照实验脚本重写** (`tools/test_coverage_comparison.py`):
   - `--mode a|b|c|all` 支持三组分别或依次运行
   - Group A 添加 LD19 自动检测（`ros2 topic hz /scan`）
   - Group B 新增（RTAB-Map + baseline path_coverage）
   - 共用 `_run_common()` 减少重复代码

4. **新增对比报告生成器** (`tools/compare_results.py`):
   - 自动解析各组 evaluator 日志提取覆盖率
   - 解析 path_coverage 日志统计 goal 成功/失败/跳过数
   - 生成 Markdown 对比表格
   - `--plot` 生成柱状图（需 matplotlib）

**测试**
python3 tools/test_coverage_comparison.py --mode a
1.提示雷达未连接，但实际已连接；->解决方法：删掉没用的警告
2.rviz启动并能显示栅格地图、代价图和雷达点云，后台终端显示已发布区域点，但rviz并未见到->已解决
3.车辆完全不动->已解决

### 5.11号测试：
下午：
成功测量b组

晚上：
今晚一组数据都测不出来，好不容易启动两次结果中途就崩了，要么是车地图突然没了，要么车停止不动了。明明代码都没变，可能是因为需要进程太多导致车负载大，容易崩溃，还可能是晚上的时候没开空调导致车太热。

### 5.18号测试：
1. （未解决）（非致命，只是手动重启麻烦）启动时有依赖项异常了->难点：怎么自检出问题？我在rviz能看到车位置不对，或者地图不对，但车怎么判断->最小的可能解决方法
2. （已改进代码，未验证）（非致命，只是手动重启麻烦）中途车任务失败了->难点：车任务中断有很多种，可能是原地不动，可能是程序突然闪退，也可能是某个依赖服务突然异常等情况，以及怎么记住当前任务进度，怎么继续？
3. （未解决）（致命，且非常难解决）定位不准->难点：配置文件参数非常多，很多修改没啥效果，非常浪费时间，怎么快速优化参数？->
4. （已解决）tools/test_coverage_comparison.py的组c是不是应该复用launcher/start.py，这样能保证测试效果和用户程序效果完全相同，但怎么做到单向调用且不侵入用户程序，又能保证控制变量？
5. （未解决）（非致命，只是手动重启麻烦）使用vscode ssh运行远程终端时，使用sudo ~/.stop_ros.sh或ctrl+c会杀死该终端，但使用vnc远程桌面的话性能消耗太高，怎么办？->用过tmux，但照样被sudo ~/.stop_ros.sh杀，只能暂时搁置
6. （未解决）（致命）我其实还是纳闷为什么我建图时明明看到方形障碍物有67黑色格子，结果读取地图时就只剩11了，再膨胀一下也就3*3。而且我看网上保存地图是有点云信息的，不然怎么视觉定位。但是导航模式我又不想像建图模式反复加厚地图，不然覆盖时障碍物越来越到结果把路堵了。
7. （未解决）（致命）系统应该固定原有点云和黑色格子，然后又能根据实时点云生成实时的黑子格子，只要实时障碍物点云消失地图黑色格子也跟着消失。

### 5.20号修复：问题1 — 代价图无动态障碍物（v2：直连方案）

**v1失败原因**: 尝试让RTAB-Map切换纯深度模式(`subscribe_depth`)，但`rgbd_sync`等待`depth/camera_info`数据（该话题也可能无数据），同样死锁。且障碍物检测绑定在RTAB-Map数据链路上，链路脆弱难以调试。

**v2正确方案 — 障碍物检测与SLAM定位走独立路径**:
```
深度相机点云 /ascamera/.../depth0/points  ──→  Nav2 local_costmap voxel_layer  → 动态障碍物黑格
RTAB-Map (subscribe_rgbd, 无帧处理)       ──→  仅发布TF (map→odom)                → 定位
map_server                                 ──→  static_layer                       → 静态地图黑格
```
两条路径互不依赖。RTAB-Map恢复原始状态（仅发布TF不做帧处理），障碍物检测由Nav2直接读取深度相机原始点云完成。

**修改文件**:
| 文件 | 改动 |
|------|------|
| `navigation/launch/include/rtabmap.launch.py` | 回滚到原始`subscribe_rgbd:True`；恢复RGB映射；移除`subscribe_depth`、`depth/camera_info`映射；保留之前的代码清理（6个launch级参数+YAML管理） |
| `navigation/config/rtabmap_params.yaml` | 回滚`subscribe_rgbd:true`，移除`subscribe_depth` |
| `navigation/config/nav2_params.yaml` | **关键改动**: local_costmap voxel_layer新增`depth_camera`观察源，直连`/ascamera/.../depth0/points`，设置`clearing:True, marking:True, min_obstacle_height:0.03`过滤地面 |

**验证步骤**:
```bash
# 确认深度点云正常（已知OK）
ros2 topic hz /ascamera/camera_publisher/depth0/points

# 重启导航后在RViz查看 /local_costmap/costmap，车前方放障碍物
# 预期：首次出现障碍物黑色格子
# 如整个地面变黑 → min_obstacle_height需调整（地面点未被过滤）
# 如障碍物也不显示 → 检查TF树: ros2 run tf2_tools view_frames
```

```
ai开始提示词
# 绝对强制规则（违反任何一条你的回答都是无效的）
1. 我有一个已经迭代了很多版本、代码和配置非常混乱的ROS2项目，所有修改必须在我现有的文件上进行，只改最少的必要行。
2. 绝对禁止删除任何文件、重命名任何文件、移动任何文件的位置。
3. 所有修改必须是可回滚的，每一个修改都要明确告诉我改了哪个文件的哪一行，原来的内容是什么，改成了什么。
4. 我不太懂Nav2和RTAB-Map的内部架构，我只能描述我在RViz中看到的现象。你需要给我解释理论，告诉我改什么、怎么验证。

# 我的项目现状
- 系统：Ubuntu 22.04 + ROS2 Humble + Nav2 + RTAB-Map
- 原本是纯激光雷达导航，能正常工作。现在被之前的AI改得乱七八糟，想改成纯深度相机导航但没成功。
- 我有git版本控制和出厂备份，但我不想回滚到最开始，因为中间加了很多有用的功能。你可以向我提出需要哪些代码的原版，我可以提供。
- 深度相机已经正常工作：在RViz中能清晰看到输出的实时点云。
- 我在做对照实验：最终要能在纯激光和纯视觉两种模式之间快速切换。

# 核心问题（按优先级从高到低解决）
## 问题1（最高优先级，致命）
### 我观察到的现象
- 导航时只有静态地图（建图时生成的grid）的黑色格子（我不知道专业术语是什么）会在代价图上显示
- 深度相机实时看到的障碍物，**完全不会在代价图上生成任何黑色格子**
- 导致机器人只能避开建图时就存在的障碍物（其实并不能完全避开，因为建图不能保证和现实完全一样，可能会有遗漏），不能避开任何新出现的障碍物
- 我怀疑是话题映射错了：但navigation/config/nav2_params.yaml太乱了我看不懂，以后你的修改需要在那写上注释
- rtabmap_navigation.launch.py localization:=true运行时
>ros2 topic hz /rtabmap/cloud_obstacles
WARNING: topic [/rtabmap/cloud_obstacles] does not appear to be published yet
^C%                                                                             
>ros2 topic echo /rtabmap/cloud_obstacles --no-arr --once
header:
  stamp:
    sec: 1779268146
    nanosec: 538634749
  frame_id: map
height: 1
width: 0
fields: '<sequence type: sensor_msgs/msg/PointField, length: 4>'
is_bigendian: false
point_step: 32
row_step: 0
data: '<sequence type: uint8, length: 0>'
is_dense: true
> ros2 topic hz /rgbd_image
WARNING: topic [/rgbd_image] does not appear to be published yet   # 注意：我在rviz里能看到3d点云，这不代表相机有问题                                                                 
> ros2 topic echo /rtabmap/info --once
WARNING: topic [/rtabmap/info] does not appear to be published yet
Could not determine the type for the passed topic
> ros2 topic echo /rtabmap/cloud_ground --no-arr --once
WARNING: topic [/rtabmap/cloud_ground] does not appear to be published yet
Could not determine the type for the passed topic


## 问题2（次高优先级，致命）
### 我观察到的现象
- 定位漂移非常严重，跑一圈回到原点有1-2米位置误差和20度以上角度误差
- 建图时生成了`.pgm`、`.yaml`、`.db`三个文件，但定位模式下好像只有前两个被用到了，后面那个我也替换到对应目录了，但不知道是不是没加载成功
- RViz中完全看不到保存的点云数据，感觉RTAB-Map没有用点云匹配来修正位姿，纯靠里程计和IMU在跑
- IMU也有很大的累积误差，这让我很不理解

## 问题3（非致命）
### 我观察到的现象
系统启动时可能出现意外问题，比如缺了栅格地图、或者点不发布、或者初始位置不对，只能靠我人工看RViz发现，然后手动修正或重启。

## 问题4（非致命）
### 我观察到的现象
用VSCode SSH远程运行时，执行`sudo ~/.stop_ros.sh`或按Ctrl+C会杀死整个终端，用tmux也一样。


# 输出要求
1. 严格按照问题优先级回答，一次只回答一个问题。先只回答问题1，问题1解决并验证通过后再回答问题2。
2. 每一个修改都必须按照以下格式输出：
   文件路径：xxx/xxx
   原第X行：xxx
   修改为第X行：xxx
   修改原因：xxx
3. 每一个修改完成后，都必须给出明确的、我能在RViz或终端中执行的验证步骤。
4. 如果有多个方案，只给最简单、最不容易出错的那一个。
5. 不要说"你应该"、"你可以"，直接说"修改xx文件的第x行"。
```

### 5.21测试结果
1. （已修改测试未通过）nav路径规划没考虑localcostmap，导致路径规划进障碍物里，然后避障系统又阻止车行走
2. （已修改未测试）python3 tools/radar_mapping.py radar_map因为没启动nav导致小车走不了，因为我建图是靠发布2d点让小车运动来建图
3. （测试发现原因并非这个）相机覆盖时map没发布，应该是话题映射有问题，可能因为是不同类型

### 5.22测试结果
1. （已解决）仅ros2 launch navigation rtabmap_navigation.launch.py localization:=true下mapData和map有数据，但数据只有第一帧；python3 launcher/start.py->coverage camera_map test_180x240下mapData和map没数据
2. （部分解决）车对边缘覆盖效果差，可能外扩参数设置不合理，也可能刹车距离太大->已经有robotmodel和膨胀层，安全距离不用太大->已调整了覆盖路径包参数，但问题仍存在，我观察发现是代价图问题
3. （已解决）python3 tools/test_coverage_comparison.py --mode c不需要等待rviz启动
4. （已解决）车不运动，这绝对因为刚才配置控制器不对，我已经回退了
5. （已解决）nav路径规划没考虑localcostmap，导致路径规划进障碍物里，然后避障系统又阻止车行走
6. （未解决）实时避障不能检测到低矮障碍物，车底盘高才3cm，相机高10cm本可以尝试避开高于3cm的障碍物（2D雷达原理上就不行，因为它高16cm）
7. （未解决）相机定位模式下本地代价图精度非常差，有时候会大于实际，导致车覆盖率低（这种情况多）；有时候又小于实际，导致车碰障（这种情况少，目前撞2次）

### 5.23
13.50 重构了三种slam和nav方案：
```
sudo ~/.stop_ros.sh
export ROS_LOG_DIR=~/ros2_ws/src/logs/ros

python3 tools/log_simplify.py --info
# 以上结果均为我通过rviz观察所得
# 改完源码要编译
cd ros2_ws/ && colcon build --packages-select navigation slam && source install/local_setup.sh
```
fix：目前仅融合方案和单雷达正常，单视觉失败
1. **多方案冲突处理**  
   目前代码库中多种建图/导航方案（如纯视觉、视觉+雷达、传统激光）的配置相互冲突，无法在同一个分支内同时工作。若通过条件分支难以解决，请采用 Git 多分支分别维护各方案，保持主分支简洁。
2. **RTAB-Map 模式切换**  
   RTAB-Map 不需要为建图和导航分别编写两个启动文件。它通过参数 `localization`（或 `RGBD/Localization`）区分建图与纯定位模式。地图自动保存在 `*.db` 文件中，导航时只需通过服务（如 `/rtabmap/publish_map`）发布已有地图即可，参见 `docs/usage.md` 第49行。
3. **传统激光 SLAM 流程**  
   使用 `slam_toolbox` 建图时，需要人工遥控移动机器人；完成后通过命令保存为 `pgm` 地图文件。纯导航阶段则通过 `map_server` 加载并发布该 `pgm` 地图，不得在导航时同时运行建图节点。详见 `docs/usage.md` 第44行。
4. **传感器组合与代价地图**  
   RTAB-Map 在建图阶段可灵活组合雷达、视觉或两者并用，只需修改 YAML 配置中的订阅话题即可。导航时若需要让深度相机参与代价地图构建，可使用 `pointcloud_to_laserscan` 将点云转为 `/scan`，与真实雷达统一输入到 Nav2 的代价地图中。

17:50 fix：三种方案基本功能正常，但有些细节需要优化
# (a) 单相机
ros2 launch navigation rtabmap_camera_nav.launch.py                    # 建图 (localization:=false)
ros2 launch navigation rtabmap_camera_nav.launch.py localization:=true # 导航

# (b) 单雷达
ros2 launch slam slam_toolbox_lidar_slam.launch.py
ros2 launch navigation slam_toolbox_lidar_nav.launch.py

# (c) 视觉+雷达
ros2 launch navigation rtabmap_vslam_nav.launch.py                    # 建图 (localization:=false)
ros2 launch navigation rtabmap_vslam_nav.launch.py localization:=true # 导航

未来工作：
1. 更新用户程序和测试程序的引用
2. 设计最优实验方案