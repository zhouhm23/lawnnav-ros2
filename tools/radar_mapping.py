#!/usr/bin/env python3
"""
radar_mapping.py — 雷达 LiDAR 建图脚本。

用法:
    python3 tools/radar_mapping.py [地图名，默认 radar_map]

流程:
    1. 启动 slam_toolbox（ros2 launch slam slam.launch.py）
    2. 用户手动遥控小车遍历区域完成建图
    3. 输入 F 并回车保存地图到 ~/.ros/maps/<地图名>.yaml (+ .pgm)
"""
import os
import shlex
import subprocess
import sys
import time
from pathlib import Path

WS_ROOT = Path(__file__).resolve().parent.parent  # ~/ros2_ws/src
MAP_BACKUP_DIR = str(Path.home() / ".ros" / "maps")


def _source_cmd() -> str:
    parts = ["source /opt/ros/humble/setup.sh"]
    ws_setup = WS_ROOT / "install" / "setup.bash"
    if ws_setup.exists():
        parts.append(f"source {shlex.quote(str(ws_setup))}")
    return " && ".join(parts)


def main():
    map_name = sys.argv[1] if len(sys.argv) > 1 else "radar_map"

    print(f"\033[36m[雷达建图]\033[0m 启动 slam_toolbox...")
    slam_proc = subprocess.Popen(
        ["bash", "-lc", f"{_source_cmd()} && ros2 launch slam slam.launch.py"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    time.sleep(5.0)
    print(f"\033[36m[雷达建图]\033[0m 启动 navigation (map:=none)...")
    nav_proc = subprocess.Popen(
        ["bash", "-lc", f"{_source_cmd()} && ros2 launch navigation navigation.launch.py map:=none"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    time.sleep(3.0)  # 等待 navigation 启动
    print(f"\033[36m[雷达建图]\033[0m 建图中 — 请遥控小车遍历区域")
    print(f"\033[36m[雷达建图]\033[0m 完成后输入 \033[1mF\033[0m 并回车保存为 '{map_name}'")
    print()

    # 等待 F 键
    while True:
        try:
            key = input().strip()
        except (EOFError, KeyboardInterrupt):
            print("\n[雷达建图] 已取消")
            slam_proc.terminate()
            nav_proc.terminate()
            try:
                slam_proc.wait(timeout=5)
                nav_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                pass
            return
        if key.upper() == "F":
            break
        print("  输入 F 并回车以保存地图")

    # 保存地图
    print(f"\033[36m[雷达建图]\033[0m 保存地图为 {map_name}...")
    os.makedirs(MAP_BACKUP_DIR, exist_ok=True)
    grid_path = os.path.join(MAP_BACKUP_DIR, map_name)
    result = subprocess.run(
        ["bash", "-lc",
         f"{_source_cmd()} && "
         f"ros2 run nav2_map_server map_saver_cli -f {shlex.quote(grid_path)}"],
        capture_output=True, text=True, timeout=30,
    )
    yaml_file = f"{grid_path}.yaml"
    if os.path.exists(yaml_file):
        print(f"\033[32m[OK]\033[0m 雷达地图已保存: {yaml_file}")
    else:
        print(f"\033[33m[WARN]\033[0m 保存可能失败，请检查 /map 话题是否有数据")
        print(f"  stdout: {result.stdout}")
        print(f"  stderr: {result.stderr}")

    # 停止 slam 和 nav
    print(f"\033[36m[雷达建图]\033[0m 停止 slam_toolbox + navigation...")
    for proc in [slam_proc, nav_proc]:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
    print(f"\033[32m[OK]\033[0m 完成")


if __name__ == "__main__":
    main()
