#!/usr/bin/env python3
"""
health_check.py — 系统健康自检模块。

独立工具，不依赖 launcher 内部状态。所有调用方（start.py、test 脚本）共用。

用法:
    python3 launcher/health_check.py               # 打印报告，返回码 0/1/2
    python3 launcher/health_check.py --json         # 输出 JSON
    python3 launcher/health_check.py --topic <t>    # 指定 topic（默认 /global_costmap/costmap）

返回码:
    0 = 健康
    1 = 警告（可继续但建议排查）
    2 = 致命（不应启动覆盖）
"""

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

# ── 阈值 ──────────────────────────────────────────────────────────
UNKNOWN_FATAL_RATIO = 0.90      # 未知占比 >90% → 致命
FREE_WARN_RATIO = 0.05          # 自由占比 <5% → 警告
OBSTACLE_WARN_RATIO = 0.80      # 障碍占比 >80% → 警告

# ── 辅助 ──────────────────────────────────────────────────────────

def _source_cmd() -> str:
    ws_root = Path(__file__).resolve().parent.parent
    parts = ["source /opt/ros/humble/setup.sh"]
    ws_setup = ws_root / "install" / "setup.bash"
    if ws_setup.exists():
        parts.append(f"source {ws_setup}")
    return " && ".join(parts)


def _ros2_topic_echo_once(topic: str, timeout: float = 5.0) -> str:
    """Return the first message from a topic as a raw string, or '' on failure."""
    cmd = (
        f"{_source_cmd()} && "
        f"ros2 topic echo --once --no-arr {topic} 2>/dev/null"
    )
    try:
        r = subprocess.run(
            ["bash", "-lc", cmd],
            capture_output=True, text=True, timeout=timeout,
        )
        return r.stdout
    except (subprocess.TimeoutExpired, Exception):
        return ""


def _parse_costmap_data(raw: str):
    """Try to extract the data array from a ros2 topic echo output.

    Returns list of ints or None.
    """
    lines = raw.splitlines()
    data_line = None
    capture = False
    buf = []
    for line in lines:
        # Look for "data:" marker
        if line.strip().startswith("data:"):
            capture = True
            # Check if inline data follows on same line
            rest = line.split("data:", 1)[1].strip()
            if rest and rest != "''":
                buf.append(rest)
            continue
        if capture:
            stripped = line.strip()
            if stripped.startswith("info:"):
                break
            if stripped and not stripped.startswith("---"):
                buf.append(stripped)
    if not buf:
        return None

    joined = " ".join(buf)
    # Remove array brackets
    joined = joined.replace("[", "").replace("]", "").replace("'", "")
    parts = joined.replace(",", " ").split()
    try:
        return [int(p) for p in parts]
    except (ValueError, TypeError):
        return None


def check_costmap(topic: str, timeout: float = 8.0) -> dict:
    """Check one costmap/grid topic and return health dict."""
    result = {
        "topic": topic,
        "available": False,
        "total_cells": 0,
        "unknown_pct": 0.0,
        "free_pct": 0.0,
        "obstacle_pct": 0.0,
        "status": "unknown",
        "diagnosis": "",
    }

    raw = _ros2_topic_echo_once(topic, timeout)
    if not raw:
        result["diagnosis"] = f"{topic} 无数据 — 话题可能未发布或无人订阅"
        result["status"] = "fatal"
        return result

    data = _parse_costmap_data(raw)
    if data is None or len(data) == 0:
        result["diagnosis"] = f"{topic} 数据为空 — 话题有发布但内容为空数组"
        result["status"] = "fatal"
        return result

    result["available"] = True
    result["total_cells"] = len(data)

    n_unknown = sum(1 for v in data if v == -1)
    n_free = sum(1 for v in data if 0 <= v <= 70)
    n_obstacle = sum(1 for v in data if v >= 100)
    n_other = len(data) - n_unknown - n_free - n_obstacle

    result["unknown_pct"] = n_unknown / len(data)
    result["free_pct"] = n_free / len(data)
    result["obstacle_pct"] = n_obstacle / len(data)

    # Diagnosis
    if result["unknown_pct"] >= UNKNOWN_FATAL_RATIO:
        result["status"] = "fatal"
        result["diagnosis"] = (
            f"costmap 几乎全为未知 ({result['unknown_pct']:.0%})。"
            "可能原因: 地图未加载 / 定位未就绪 / RTAB-Map 未积累数据。"
            "建议: 检查 map_server 是否 activate、TF 树 odom→map 是否连通。"
        )
    elif result["free_pct"] < FREE_WARN_RATIO:
        result["status"] = "warn"
        result["diagnosis"] = (
            f"costmap 几乎无自由空间 ({result['free_pct']:.1%})。"
            "可能原因: inflation/障碍物层配置过大 / 传感器数据异常。"
            "建议: 检查 nav2_params.yaml 中 inflation_radius 和 obstacle_max_range。"
        )
    elif result["obstacle_pct"] >= OBSTACLE_WARN_RATIO:
        result["status"] = "warn"
        result["diagnosis"] = (
            f"costmap 障碍物占比异常高 ({result['obstacle_pct']:.0%})。"
            "可能原因: 传感器噪声 / TF 树断裂 / 点云坐标错误。"
            "建议: 检查 TF 树 odom→base_footprint→sensor 是否完整。"
        )
    else:
        result["status"] = "ok"
        result["diagnosis"] = (
            f"costmap 正常: 未知 {result['unknown_pct']:.0%}, "
            f"自由 {result['free_pct']:.0%}, 障碍 {result['obstacle_pct']:.0%}"
        )

    return result


# ── 主入口 ──────────────────────────────────────────────────────────

def main():
    p = argparse.ArgumentParser(description="ROS2 系统健康自检")
    p.add_argument("--topic", default="/global_costmap/costmap",
                   help="要检查的 costmap topic")
    p.add_argument("--json", action="store_true", help="输出 JSON")
    args = p.parse_args()

    results = []
    # 1. Check primary costmap
    r = check_costmap(args.topic)
    results.append(r)

    # 2. Also check /rtabmap/grid_map if costmap is unhealthy
    if r["status"] != "ok" and args.topic != "/rtabmap/grid_map":
        r2 = check_costmap("/rtabmap/grid_map", timeout=5.0)
        results.append(r2)

    if args.json:
        print(json.dumps(results, ensure_ascii=False, indent=2))
    else:
        # Human-readable output
        for res in results:
            icon = {"ok": "\033[32m✓\033[0m", "warn": "\033[33m⚠\033[0m", "fatal": "\033[31m✗\033[0m"}
            print(f"  {icon.get(res['status'], '?')} {res['topic']}: {res['diagnosis']}")

    # Return code: worst status
    statuses = [r["status"] for r in results]
    if "fatal" in statuses:
        sys.exit(2)
    elif "warn" in statuses:
        sys.exit(1)
    else:
        sys.exit(0)


if __name__ == "__main__":
    main()
