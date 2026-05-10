#!/usr/bin/env python3
"""
test_coverage_comparison.py — 覆盖算法对照实验。

创新组: RTAB-Map 视觉 SLAM + 改进 path_coverage (retry/try-except/costmap_wait)
对照组: LiDAR 静态地图 + 原始 path_coverage (无鲁棒性改进)

用法:
    python3 tools/test_coverage_comparison.py --innovation   # 仅创新组
    python3 tools/test_coverage_comparison.py --baseline     # 仅对照组
    python3 tools/test_coverage_comparison.py --all          # 全部（默认）

前置条件:
    - 已用 launcher/start.py 建图并 save test_map
    - 对照组需 LD19 激光雷达已连接
    - 确保 ~/.ros/maps/test_map.yaml 和 .pgm 存在
"""

import argparse
import os
import shlex
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

# ── 路径常量 ──────────────────────────────────────────────────────────
SCRIPT_DIR = Path(__file__).resolve().parent
LAUNCHER_DIR = SCRIPT_DIR / ".." / "launcher"
WS_ROOT = SCRIPT_DIR / ".."
MAP_BACKUP_DIR = str(Path.home() / ".ros" / "maps")
RTABMAP_DB = str(Path.home() / ".ros" / "rtabmap.db")
LOG_DIR = str(Path.home() / "ros2_ws" / "src" / "logs" / "comparison")
REGION_FILE = str(LAUNCHER_DIR / "regions" / "test_180x240.yaml")
SLAM_MAPS_DIR = str(WS_ROOT / "slam" / "maps")
DEFAULT_MAP = "test_map"


def _source_cmd() -> str:
    ros_setup = "/opt/ros/humble/setup.sh"
    if not os.path.exists(ros_setup):
        ros_setup = "/opt/ros/humble/local_setup.sh"
    parts = [f"source {shlex.quote(ros_setup)}"]
    ws_setup = WS_ROOT / "install" / "setup.bash"
    if ws_setup.exists():
        parts.append(f"source {shlex.quote(str(ws_setup))}")
    return " && ".join(parts)


def _info(msg: str) -> None:
    print(f"\033[36m[INFO]\033[0m {msg}")


def _ok(msg: str) -> None:
    print(f"\033[32m[OK]\033[0m {msg}")


def _warn(msg: str) -> None:
    print(f"\033[33m[WARN]\033[0m {msg}")


def _stop_ros() -> None:
    stop_script = Path.home() / ".stop_ros.sh"
    if stop_script.exists():
        subprocess.call(["bash", str(stop_script)],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(1.0)


def _ensure_test_map() -> bool:
    """确保 test_map 在 slam/maps/ 存在（对照组需要）。"""
    yaml_dst = os.path.join(SLAM_MAPS_DIR, f"{DEFAULT_MAP}.yaml")
    pgm_dst = os.path.join(SLAM_MAPS_DIR, f"{DEFAULT_MAP}.pgm")
    if os.path.exists(yaml_dst):
        return True

    yaml_src = os.path.join(MAP_BACKUP_DIR, f"{DEFAULT_MAP}.yaml")
    pgm_src = os.path.join(MAP_BACKUP_DIR, f"{DEFAULT_MAP}.pgm")
    if not os.path.exists(yaml_src):
        _warn(f"未找到 {yaml_src}，请先在 launcher 中 mapping → save {DEFAULT_MAP}")
        return False

    os.makedirs(SLAM_MAPS_DIR, exist_ok=True)
    shutil.copy2(yaml_src, yaml_dst)
    if os.path.exists(pgm_src):
        shutil.copy2(pgm_src, pgm_dst)
    _ok(f"地图 {DEFAULT_MAP} 已复制到 {SLAM_MAPS_DIR}/")
    return True


def run_innovation() -> None:
    """创新组: RTAB-Map visual SLAM + 改进 path_coverage。"""
    print()
    print("\033[1;32m╔══════════════════════════════════════════╗\033[0m")
    print("\033[1;32m║  创新组: RTAB-Map + 改进 path_coverage  ║\033[0m")
    print("\033[1;32m╚══════════════════════════════════════════╝\033[0m")
    print()
    input("按 Enter 启动 innovation 测试（确保已 stop 所有旧进程）...")

    _stop_ros()
    os.makedirs(LOG_DIR, exist_ok=True)

    _info("启动 navigation (RTAB-Map, localization:=true)...")
    nav = subprocess.Popen(
        ["bash", "-lc",
         f"{_source_cmd()} && "
         "ros2 launch navigation rtabmap_navigation.launch.py localization:=true"],
        stdout=open(os.path.join(LOG_DIR, "innovation_nav.log"), "w"),
        stderr=subprocess.STDOUT,
    )
    time.sleep(5.0)

    _info("启动 RViz...")
    rviz = subprocess.Popen(
        ["bash", "-lc",
         f"{_source_cmd()} && "
         "ros2 launch navigation rviz_rtabmap_navigation.launch.py"],
        stdout=open(os.path.join(LOG_DIR, "innovation_rviz.log"), "w"),
        stderr=subprocess.STDOUT,
    )
    time.sleep(3.0)

    # map_server
    grid_yaml = os.path.join(MAP_BACKUP_DIR, f"{DEFAULT_MAP}.yaml")
    if os.path.exists(grid_yaml):
        _info("启动 map_server...")
        subprocess.Popen(
            ["bash", "-lc",
             f"{_source_cmd()} && "
             f"ros2 run nav2_map_server map_server "
             f"--ros-args -p yaml_filename:={shlex.quote(grid_yaml)}"],
            stdout=open(os.path.join(LOG_DIR, "innovation_mapserver.log"), "w"),
            stderr=subprocess.STDOUT,
        )
        time.sleep(3.0)
        subprocess.run(
            f"{_source_cmd()} && "
            "ros2 lifecycle set /map_server configure && "
            "ros2 lifecycle set /map_server activate",
            shell=True, executable="/bin/bash",
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            timeout=10,
        )

    _info("等待 costmap 稳定 (30s)...")
    time.sleep(30.0)

    _info("启动 path_coverage (改进版)...")
    pc = subprocess.Popen(
        ["bash", "-lc",
         f"{_source_cmd()} && "
         "ros2 launch path_coverage path_coverage.launch.py"],
        stdout=open(os.path.join(LOG_DIR, "innovation_pathcoverage.log"), "w"),
        stderr=subprocess.STDOUT,
    )
    _info("启动 coverage_evaluator...")
    ev = subprocess.Popen(
        ["bash", "-lc",
         f"{_source_cmd()} && "
         "ros2 launch coverage_evaluator coverage_evaluator.launch.py"],
        stdout=open(os.path.join(LOG_DIR, "innovation_evaluator.log"), "w"),
        stderr=subprocess.STDOUT,
    )

    _info("等待节点就绪 (15s)...")
    time.sleep(15.0)

    _info("发布覆盖区域...")
    pub_script = str(LAUNCHER_DIR / "publish_region.py")
    rc = subprocess.run(
        ["python3", pub_script, "--file", REGION_FILE, "--wait", "3"],
        timeout=30,
    )
    if rc.returncode != 0:
        _warn(f"区域发布失败 (exit={rc.returncode})")
        return

    _ok("创新组就绪 — 等待覆盖完成（可 Ctrl+C 提前结束，进程会自动清理）...")
    print()
    try:
        nav.wait()
    except KeyboardInterrupt:
        pass
    finally:
        for p in [pc, ev, rviz, nav]:
            if p.poll() is None:
                p.terminate()
                try:
                    p.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    p.kill()
    print()
    _ok("创新组完成 ✓")


def run_baseline() -> None:
    """对照组: LiDAR 静态地图 + 原始 path_coverage。"""
    print()
    print("\033[1;33m╔══════════════════════════════════════════╗\033[0m")
    print("\033[1;33m║  对照组: LiDAR + 原始 path_coverage     ║\033[0m")
    print("\033[1;33m╚══════════════════════════════════════════╝\033[0m")
    print()
    print("  ⚠ 需要 LD19 激光雷达已连接")
    input("按 Enter 启动 baseline 测试（确保已 stop 所有旧进程）...")

    _stop_ros()
    os.makedirs(LOG_DIR, exist_ok=True)

    if not _ensure_test_map():
        return

    _info("启动 navigation (LiDAR 静态地图)...")
    nav = subprocess.Popen(
        ["bash", "-lc",
         f"{_source_cmd()} && "
         f"ros2 launch navigation navigation.launch.py map:={DEFAULT_MAP}"],
        stdout=open(os.path.join(LOG_DIR, "baseline_nav.log"), "w"),
        stderr=subprocess.STDOUT,
    )
    time.sleep(5.0)

    _info("启动 RViz (厂家默认)...")
    rviz = subprocess.Popen(
        ["bash", "-lc",
         f"{_source_cmd()} && "
         "ros2 launch navigation rviz_navigation.launch.py"],
        stdout=open(os.path.join(LOG_DIR, "baseline_rviz.log"), "w"),
        stderr=subprocess.STDOUT,
    )
    time.sleep(3.0)

    _info("等待 costmap 稳定 (20s)...")
    time.sleep(20.0)

    _info("启动 path_coverage (原始版)...")
    pc = subprocess.Popen(
        ["bash", "-lc",
         f"{_source_cmd()} && "
         "ros2 launch path_coverage path_coverage_baseline.launch.py"],
        stdout=open(os.path.join(LOG_DIR, "baseline_pathcoverage.log"), "w"),
        stderr=subprocess.STDOUT,
    )
    _info("启动 coverage_evaluator...")
    ev = subprocess.Popen(
        ["bash", "-lc",
         f"{_source_cmd()} && "
         "ros2 launch coverage_evaluator coverage_evaluator.launch.py"],
        stdout=open(os.path.join(LOG_DIR, "baseline_evaluator.log"), "w"),
        stderr=subprocess.STDOUT,
    )

    _info("等待节点就绪 (10s)...")
    time.sleep(10.0)

    _info("发布覆盖区域...")
    pub_script = str(LAUNCHER_DIR / "publish_region.py")
    rc = subprocess.run(
        ["python3", pub_script, "--file", REGION_FILE, "--wait", "3"],
        timeout=30,
    )
    if rc.returncode != 0:
        _warn(f"区域发布失败 (exit={rc.returncode})")
        return

    _ok("对照组就绪 — 等待覆盖完成...")
    print()
    try:
        nav.wait()
    except KeyboardInterrupt:
        pass
    finally:
        for p in [pc, ev, rviz, nav]:
            if p.poll() is None:
                p.terminate()
                try:
                    p.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    p.kill()
    print()
    _ok("对照组完成 ✓")


# ═══════════════════════════════════════════════════════════════════════════
# main
# ═══════════════════════════════════════════════════════════════════════════

def main() -> None:
    parser = argparse.ArgumentParser(description="覆盖算法对照实验")
    parser.add_argument("--mode", choices=["innovation", "baseline", "all"],
                        default="all",
                        help="innovation: 创新组 | baseline: 对照组 | all: 全部")
    args = parser.parse_args()

    def on_sigint(sig, _frame):
        print("\n\033[33m中断信号\033[0m — 进程将自动清理")
        sys.exit(130)

    signal.signal(signal.SIGINT, on_sigint)

    if args.mode in ("innovation", "all"):
        run_innovation()
        if args.mode == "all":
            print("\n" + "=" * 50 + "\n")

    if args.mode in ("baseline", "all"):
        run_baseline()


if __name__ == "__main__":
    main()
