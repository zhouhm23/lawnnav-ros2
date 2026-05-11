#!/usr/bin/env python3
"""
compare_results.py — 三组消融实验对比报告生成器。

从 logs/comparison/ 提取各组 evaluator 日志，自动生成对比表格和覆盖率曲线。

用法:
    python3 tools/compare_results.py                        # 汇总所有日志
    python3 tools/compare_results.py --plot                 # 同时生成图表
    python3 tools/compare_results.py --output report.md     # 输出到指定文件
"""

import argparse
import os
import re
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

LOG_DIR = str(Path.home() / "ros2_ws" / "src" / "logs" / "comparison")

GROUP_INFO = {
    "group_a_baseline":    ("A (LiDAR + 原始)",       "传统基准"),
    "group_b_ablation":    ("B (RTAB-Map + 原始)",    "消融组"),
    "group_c_innovation":  ("C (RTAB-Map + 改进)",    "创新组"),
}

# 匹配 coverage_evaluator 的覆盖率输出行
COV_RE = re.compile(
    r"\[coverage_evaluator\]:\s+Coverage:\s+([0-9.]+)%\s+\((\d+)/(\d+)\s+cells\)"
)
TIMESTAMP_RE = re.compile(r"\[(\d+\.\d+)\]")

# 匹配 path_coverage 的关键事件
CELL_OK_RE = re.compile(r"Goal succeeded!")
CELL_FAIL_RE = re.compile(r"(Cell \d+ failed|skipping)")
CELL_SKIP_RE = re.compile(r"waypoint.*failed.*skipping")

# 匹配最终覆盖率（evaluator 日志中含 "completed" 或 "finished" 的行后的最后一帧）
COMPLETED_RE = re.compile(r"(Boustrophedon Decomposition completed|this signifies the end)")


def parse_evaluator_log(log_path: str) -> List[Tuple[float, float, int]]:
    """解析 evaluator 日志，返回 [(timestamp, coverage_pct, covered_cells), ...]"""
    records = []
    if not os.path.exists(log_path):
        return records
    with open(log_path, "r", errors="replace") as f:
        for line in f:
            m = COV_RE.search(line)
            if m:
                pct = float(m.group(1))
                cells = int(m.group(2))
                total = int(m.group(3))
                # 尝试提取时间戳
                tm = TIMESTAMP_RE.search(line)
                t = float(tm.group(1)) if tm else 0.0
                records.append((t, pct, cells))
    return records


def parse_pathcoverage_log(log_path: str) -> Dict:
    """解析 path_coverage 日志，返回事件统计."""
    stats = {"goals_succeeded": 0, "cells_failed": 0, "waypoints_skipped": 0, "completed": False}
    if not os.path.exists(log_path):
        return stats
    with open(log_path, "r", errors="replace") as f:
        for line in f:
            if CELL_OK_RE.search(line):
                stats["goals_succeeded"] += 1
            if CELL_FAIL_RE.search(line):
                stats["cells_failed"] += 1
            if CELL_SKIP_RE.search(line):
                stats["waypoints_skipped"] += 1
            if COMPLETED_RE.search(line):
                stats["completed"] = True
    return stats


def final_coverage(records: List[Tuple[float, float, int]]) -> Tuple[float, int]:
    """返回最终覆盖率."""
    if not records:
        return 0.0, 0
    return records[-1][1], records[-1][2]


def generate_report(output_file: Optional[str] = None, plot: bool = False) -> str:
    """生成对比报告."""
    lines = []
    lines.append("# 覆盖算法三组消融实验对比报告")
    lines.append("")
    lines.append("## 各组最终指标")
    lines.append("")
    lines.append("| 组 | 传感器 | 算法 | 最终覆盖率 | 覆盖格子 | Goal成功 | Cell失败 | WP跳过 | 完成状态 |")
    lines.append("|:---|:---|:---|:---:|:---:|:---:|:---:|:---:|:---|")

    for prefix, (label, desc) in GROUP_INFO.items():
        ev_log = os.path.join(LOG_DIR, f"{prefix}_evaluator.log")
        pc_log = os.path.join(LOG_DIR, f"{prefix}_pathcoverage.log")

        records = parse_evaluator_log(ev_log)
        pc_stats = parse_pathcoverage_log(pc_log)
        cov_pct, cov_cells = final_coverage(records)

        # 推导传感器类型
        if "baseline" in prefix:  # group_a
            sensor = "LiDAR"
            algo = "原始"
        elif "ablation" in prefix:
            sensor = "RTAB-Map"
            algo = "原始"
        else:
            sensor = "RTAB-Map"
            algo = "改进"

        status = "✅ 完成" if pc_stats["completed"] else "❌ 未完成/中断"
        lines.append(
            f"| {label} | {sensor} | {algo} | "
            f"{cov_pct:.1f}% | {cov_cells} | "
            f"{pc_stats['goals_succeeded']} | {pc_stats['cells_failed']} | "
            f"{pc_stats['waypoints_skipped']} | {status} |"
        )

    lines.append("")
    lines.append("## 各组覆盖率时间序列")
    lines.append("")
    lines.append("```")

    for prefix, (label, _desc) in GROUP_INFO.items():
        ev_log = os.path.join(LOG_DIR, f"{prefix}_evaluator.log")
        records = parse_evaluator_log(ev_log)
        if not records:
            lines.append(f"{label}: 无数据")
            continue
        # 显示前10帧和最后5帧
        show = records[:10]
        if len(records) > 15:
            lines.append(f"{label}: ... (省略中间 {len(records)-15} 帧) ...")
            show += records[-5:]
        for t, pct, cells in show:
            lines.append(f"  {label}  t={t:.0f}s  {pct:.1f}%  ({cells} cells)")
        if records:
            lines.append(f"  {label}  最终: {records[-1][1]:.1f}%")
        lines.append("")

    lines.append("```")
    lines.append("")
    lines.append("## 论证分析")
    lines.append("")

    # 收集各组最终覆盖率
    covs = {}
    for prefix, (label, _desc) in GROUP_INFO.items():
        ev_log = os.path.join(LOG_DIR, f"{prefix}_evaluator.log")
        records = parse_evaluator_log(ev_log)
        covs[prefix] = final_coverage(records)[0]

    a_cov = covs.get("group_a_baseline", 0)
    b_cov = covs.get("group_b_ablation", 0)
    c_cov = covs.get("group_c_innovation", 0)

    if b_cov > 0:
        lines.append(f"- **A vs B** (仅换传感器): 覆盖率从 {a_cov:.1f}% "
                      f"{'下降' if b_cov < a_cov else '变化'}到 {b_cov:.1f}%，"
                      f"差值 {abs(a_cov - b_cov):.1f}%")
    if c_cov > 0:
        lines.append(f"- **B vs C** (加入算法改进): 覆盖率从 {b_cov:.1f}% "
                      f"{'提升' if c_cov > b_cov else '变化'}到 {c_cov:.1f}%，"
                      f"差值 {abs(c_cov - b_cov):.1f}%")
    if a_cov > 0 and c_cov > 0:
        lines.append(f"- **A vs C** (完整方案对比): 覆盖率 {a_cov:.1f}% → {c_cov:.1f}%，"
                      f"差值 {abs(a_cov - c_cov):.1f}%")

    # 碰撞数据提示
    lines.append("")
    lines.append("> ⚠ 碰撞次数需人工观察记录（日志中无可自动提取的碰撞信号）。")
    lines.append("> 请根据实验时人工记录的碰撞次数手动填入上表。")

    report = "\n".join(lines)

    if output_file:
        with open(output_file, "w") as f:
            f.write(report)
        print(f"报告已写入: {output_file}")
    else:
        print(report)

    if plot:
        try:
            _generate_plot(covs)
        except ImportError:
            print("⚠ matplotlib 未安装，跳过图表生成")

    return report


def _generate_plot(covs: Dict[str, float]) -> None:
    """生成覆盖率柱状对比图."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    labels = ["A\nLiDAR+原始", "B\n视觉+原始", "C\n视觉+改进"]
    values = [
        covs.get("group_a_baseline", 0),
        covs.get("group_b_ablation", 0),
        covs.get("group_c_innovation", 0),
    ]
    colors = ["#3498db", "#e74c3c", "#2ecc71"]

    fig, ax = plt.subplots(figsize=(8, 5))
    bars = ax.bar(labels, values, color=colors, edgecolor="black")
    ax.set_ylabel("Coverage Rate (%)")
    ax.set_title("Three-Group Ablation Experiment: Coverage Comparison")
    ax.set_ylim(0, max(values) * 1.3 if max(values) > 0 else 100)

    for bar, val in zip(bars, values):
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 1,
                f"{val:.1f}%", ha="center", fontweight="bold")

    plot_path = os.path.join(LOG_DIR, "comparison_plot.png")
    plt.tight_layout()
    plt.savefig(plot_path, dpi=150)
    print(f"图表已保存: {plot_path}")


def main() -> None:
    parser = argparse.ArgumentParser(description="三组消融实验对比报告")
    parser.add_argument("--plot", action="store_true", help="生成柱状图")
    parser.add_argument("--output", "-o", default=None, help="输出文件路径")
    args = parser.parse_args()

    if not os.path.isdir(LOG_DIR):
        print(f"日志目录不存在: {LOG_DIR}")
        print("请先运行 test_coverage_comparison.py")
        sys.exit(1)

    generate_report(output_file=args.output, plot=args.plot)


if __name__ == "__main__":
    main()
