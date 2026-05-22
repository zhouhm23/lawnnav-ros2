#!/usr/bin/env python3
"""
radar_mapping.py — 雷达 LiDAR 建图脚本（含键盘遥控）。

用法:
    python3 tools/radar_mapping.py [地图名，默认 radar_map]

遥控: i=前进 ,=后退 j=左转 l=右转 k=停 Ctrl+C=退出遥控
保存: 在此终端输入 F 并回车
"""
import os
import shlex
import subprocess
import sys
import time
from pathlib import Path

WS_ROOT = Path(__file__).resolve().parent.parent
MAP_BACKUP_DIR = str(Path.home() / ".ros" / "maps")


def _source_cmd() -> str:
    parts = ["source /opt/ros/humble/setup.sh"]
    ws_setup = WS_ROOT / "install" / "setup.bash"
    if ws_setup.exists():
        parts.append(f"source {shlex.quote(str(ws_setup))}")
    return " && ".join(parts)


def main():
    map_name = sys.argv[1] if len(sys.argv) > 1 else "radar_map"

    # 1. 启动 SLAM
    print(f"\033[36m[雷达建图]\033[0m 启动 slam_toolbox...")
    slam_proc = subprocess.Popen(
        ["bash", "-lc", f"{_source_cmd()} && ros2 launch slam slam.launch.py"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    time.sleep(8.0)

    # 2. 提示用户新开终端运行键盘遥控
    print(f"\033[36m[雷达建图]\033[0m 建图中")
    print(f"  ⚠ 请\033[1m新开终端\033[0m运行键盘遥控（i=前进 ,=后退 j=左转 l=右转 k=停 Ctrl+C=退出）:")
    print(f"  \033[33mros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel\033[0m")
    print(f"  建图完成后在此处输入 \033[1mF\033[0m 并回车保存")
    print()

    # 3. 等待 F 保存
    while True:
        try:
            key = input().strip()
        except (EOFError, KeyboardInterrupt):
            print("\n[雷达建图] 已取消")
            slam_proc.terminate()
            try:
                slam_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                pass
            return
        if key.upper() == "F":
            break
        print("  输入 F 并回车以保存地图")

    # 3. 保存地图
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

    # 4. 停止 slam
    print(f"\033[36m[雷达建图]\033[0m 停止 slam_toolbox...")
    slam_proc.terminate()
    try:
        slam_proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        slam_proc.kill()
    print(f"\033[32m[OK]\033[0m 完成")


if __name__ == "__main__":
    main()
