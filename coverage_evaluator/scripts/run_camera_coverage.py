#!/usr/bin/env python3
"""
run_camera_coverage.py — 离线俯视视频覆盖分析 CLI 入口。

用法:
    # Linux (ROS2 环境，已 colcon build):
    ros2 run coverage_evaluator run_camera_coverage --video test.mp4 --mask mask.png

    # Windows / 任意平台 (把 camera_coverage.py 和本文件放同一目录):
    python run_camera_coverage.py --video test.mp4 --mask mask.png --visualize

依赖: opencv-python, numpy, matplotlib (pip install opencv-python matplotlib numpy)
"""

import argparse
import sys
import os

# ── 智能导入：优先 ROS2 包路径，回退到同目录独立文件 ────────────────────
try:
    from coverage_evaluator.camera_coverage import (
        CameraCoverageAnalyzer,
        CameraCoverageConfig,
    )
except ImportError:
    # 独立运行：从同目录导入 camera_coverage.py
    _script_dir = os.path.dirname(os.path.abspath(__file__))
    if _script_dir not in sys.path:
        sys.path.insert(0, _script_dir)
    from camera_coverage import (  # type: ignore
        CameraCoverageAnalyzer,
        CameraCoverageConfig,
    )


def main():
    parser = argparse.ArgumentParser(
        description="基于俯视摄像的覆盖真值离线分析")
    parser.add_argument("--video", required=True,
                        help="俯视视频文件路径 (.mp4)")
    parser.add_argument("--mask", required=True,
                        help="PS 蒙版 PNG 路径（白色=可通行，黑色=不可通行）")
    parser.add_argument("--output", default="./coverage_results",
                        help="输出目录 (默认: ./coverage_results)")
    parser.add_argument("--visualize", action="store_true",
                        help="生成可视化图像（coverage_overlay.png + coverage_curve.png）")
    parser.add_argument("--start-frame", type=int, default=0,
                        help="起始帧 (默认: 0)")
    parser.add_argument("--end-frame", type=int, default=-1,
                        help="结束帧 (默认: -1 = 到结尾)")
    parser.add_argument("--frame-skip", type=int, default=1,
                        help="处理帧间隔，默认每帧处理")
    parser.add_argument("--coverage-radius", type=float, default=0.12,
                        help="覆盖半径 m (默认: 0.12)")
    parser.add_argument("--resolution", type=float, default=0.005,
                        help="网格分辨率 m (默认: 0.005)")
    parser.add_argument("--robot-id", type=int, default=4,
                        help="车顶 ArUco ID (默认: 4)")
    args = parser.parse_args()

    # 检查文件存在
    if not os.path.exists(args.video):
        print(f"错误: 视频文件不存在: {args.video}")
        sys.exit(1)
    if not os.path.exists(args.mask):
        print(f"错误: 蒙版文件不存在: {args.mask}")
        sys.exit(1)

    # 构建配置
    config = CameraCoverageConfig(
        coverage_radius=args.coverage_radius,
        resolution=args.resolution,
        frame_skip=args.frame_skip,
        robot_id=args.robot_id,
    )

    # 执行分析
    analyzer = CameraCoverageAnalyzer(config)
    try:
        analyzer.analyze(args.video, args.mask)
    except Exception as e:
        print(f"分析失败: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

    # 保存结果
    prefix = os.path.splitext(os.path.basename(args.video))[0]
    analyzer.save_results(args.output, prefix)

    # 可视化
    if args.visualize:
        print("生成可视化...")
        analyzer.generate_visualizations(args.output, prefix)

    print("\n完成 ✓")


if __name__ == "__main__":
    main()
