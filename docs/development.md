# 开发历程与架构说明

## 核心模块说明

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
