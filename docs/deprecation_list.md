# 废弃文件清单 — 三种 SLAM/导航方案重构

## 说明
以下文件已被新的模块化方案替代。请在**测试通过后**手动删除。
每个文件顶部均已添加 `# DEPRECATED:` 注释标注替代文件。

---

## SLAM 层

| 废弃文件 | 替代文件 | 删除条件 |
|---------|---------|---------|
| `slam/launch/rtabmap_slam.launch.py` | `rtabmap_camera_slam.launch.py` | (a) 相机建图测试通过 |
| `slam/launch/slam.launch.py` | `slam_toolbox_lidar_slam.launch.py` | (b) 雷达建图测试通过 |
| `slam/launch/rtabmap_slam0.launch.py` | —（实验残留） | 确认无引用后随时删除 |

## 导航层

| 废弃文件 | 替代文件 | 删除条件 |
|---------|---------|---------|
| `navigation/launch/rtabmap_navigation.launch.py` | `rtabmap_camera_nav.launch.py` | (a) 相机导航测试通过 |
| `navigation/launch/navigation.launch.py` | `slam_toolbox_lidar_nav.launch.py` | (b) 雷达导航测试通过 |

## 参数文件

| 废弃文件 | 替代文件 | 删除条件 |
|---------|---------|---------|
| `navigation/config/nav2_params.yaml` | `nav2_params_camera.yaml` / `_lidar.yaml` / `_vslam.yaml` | 三种方案均测试通过 |
| `navigation/config/rtabmap_params.yaml` | `rtabmap_params_camera.yaml` / `_vslam.yaml` | (a)(c) 均测试通过 |

## 永久保留（禁止删除）

| 文件 | 原因 |
|-----|------|
| `slam/launch/rtabmap_slam.launch_bak.py` | 出厂原始源码，作永久参考 |

## 其他引用检查清单

以下文件可能引用了旧 launch 文件名，测试前需更新：

- [ ] `launcher/start.py` — 硬编码 `navigation rtabmap_navigation.launch.py`
- [ ] `tools/test1_slam_nav_test.py` — 可能引用旧文件名
- [ ] `tools/test2_nav_cte_and_obstacle_test.py` — 可能引用旧文件名
- [ ] `tools/goal_all.py` — 可能引用旧文件名

---

> 生成日期: 2026-05-23
> 重构说明: 将单相机(a)/单雷达(b)/视觉+雷达(c)三种方案模块化分离，各自独立入口。
