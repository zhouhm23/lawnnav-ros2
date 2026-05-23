#!/usr/bin/env python3
"""
goal_tracker.py — 导航目标不可达率统计（供 path_coverage_node 系列复用）

记录每次 NavigateToPose / NavigateThroughPoses 的结果，
在任务正常完成时输出 Invalid Goal Rate 统计 CSV 到 tools/results/。

用法（在 MapDrive 中）:
    from path_coverage_ros2.scripts.test.goal_tracker import GoalTracker

    # __init__ 中创建（指定算法类型）
    self._goal_tracker = GoalTracker(algo_type="improved")

    # navigate_to_pose 返回前记录（传入到达位姿）
    self._goal_tracker.record_to_pose(
        target_x, target_y, self.x, self.y, success, reason)

    # navigate_through_poses 返回前记录
    self._goal_tracker.record_through_poses(
        n_pts, x0, y0, xn, yn, self.x, self.y, success, reason)

    # 正常完成时
    self._goal_tracker.finish()

    # 用户手动退出时（不写文件）
    self._goal_tracker.abort()
"""

import csv
import os
import time
from pathlib import Path


class GoalTracker:
    """导航目标追踪器。

    两份输出（仅 finish() 时写入，abort() 不写）：
    - 汇总 CSV: tools/results/invalid_goal_log.csv（每轮追加一行）
    - 详情 CSV: tools/results/invalid_goal_log/{algo}_{ts}.csv（逐点）
    """

    DEFAULT_CSV_DIR = os.path.join(
        os.path.expanduser("~"), "ros2_ws", "src", "tools", "results")

    # Nav2 GoalStatus 枚举
    _STATUS_LABELS = {
        0: "UNKNOWN", 1: "ACCEPTED", 2: "EXECUTING",
        3: "CANCELING", 4: "SUCCEEDED",
        5: "CANCELED", 6: "ABORTED",
    }

    def __init__(self, algo_type: str = "improved", csv_dir: str = ""):
        self._algo_type = algo_type
        self._csv_dir = csv_dir or self.DEFAULT_CSV_DIR

        self._total = 0
        self._failed = 0
        self._reason_counts = {
            "timeout": 0, "blocked": 0, "rejected": 0,
            "server_unavailable": 0, "unknown": 0,
        }

        # 逐点详情: [(seq, target_x, target_y, reached_x, reached_y,
        #              elapsed_s, status, note), ...]
        self._goals = []
        self._start_time = None

    # ── 公共 API ───────────────────────────────────────────────────

    def record_to_pose(self, target_x: float, target_y: float,
                       reached_x, reached_y,
                       success: bool, reason: str = ""):
        self._append_goal(target_x, target_y, reached_x, reached_y,
                          success, reason)

    def record_through_poses(self, n_pts: int,
                             x0: float, y0: float,
                             xn: float, yn: float,
                             reached_x, reached_y,
                             success: bool, reason: str = ""):
        label = f"NavThrough({n_pts}pts,->{xn:.2f},{yn:.2f})"
        self._append_goal(x0, y0, reached_x, reached_y,
                          success, reason, label=label)

    def finish(self):
        if self._total == 0:
            return
        self._print_summary()
        self._write_detail_csv()
        self._write_summary_csv()

    def abort(self):
        if self._total == 0:
            return
        self._print_summary()

    # ── 内部 ───────────────────────────────────────────────────────

    def _append_goal(self, target_x, target_y,
                     reached_x, reached_y,
                     success, reason, label=""):
        if self._start_time is None:
            self._start_time = time.time()
        elapsed = time.time() - self._start_time
        seq = self._total + 1
        self._total += 1

        rx = _fmt(reached_x)
        ry = _fmt(reached_y)

        if not success:
            self._failed += 1
            readable = _enrich(reason, self._STATUS_LABELS)
            if label:
                readable = f"{label}: {readable}"
            self._goals.append(
                (seq, target_x, target_y, rx, ry,
                 f"{elapsed:.1f}", "FAIL", readable))
            self._classify(reason)
        else:
            self._goals.append(
                (seq, target_x, target_y, rx, ry,
                 f"{elapsed:.1f}", "OK", ""))

    def _classify(self, reason: str):
        r = reason.lower()
        if "timeout" in r or "超时" in r or "canceled" in r:
            self._reason_counts["timeout"] += 1
        elif "reject" in r or "拒绝" in r:
            self._reason_counts["rejected"] += 1
        elif ("block" in r or "obstacle" in r
              or "障碍" in r or "无法到达" in r or "aborted" in r):
            self._reason_counts["blocked"] += 1
        elif "server" in r and "unavail" in r:
            self._reason_counts["server_unavailable"] += 1
        else:
            self._reason_counts["unknown"] += 1

    def _print_summary(self):
        rate = (self._failed / self._total * 100.0) if self._total > 0 else 0.0
        print("\n" + "=" * 60)
        print(f"  [{self._algo_type}] Invalid Goal Rate: "
              f"{self._failed}/{self._total} = {rate:.1f}%")
        print(f"  timeout={self._reason_counts['timeout']},"
              f" blocked={self._reason_counts['blocked']},"
              f" rejected={self._reason_counts['rejected']},"
              f" server_unavail={self._reason_counts['server_unavailable']},"
              f" unknown={self._reason_counts['unknown']}")
        failures = [g for g in self._goals if g[6] == "FAIL"]
        if failures:
            print("  Failures (first 10):")
            for g in failures[:10]:
                print(f"    #{g[0]} tgt=({g[1]:.2f},{g[2]:.2f})"
                      f" rch=({g[3]},{g[4]}) @{g[5]}s — {g[7]}")
            if len(failures) > 10:
                print(f"    ... and {len(failures) - 10} more")
        print("=" * 60 + "\n")

    def _write_summary_csv(self):
        os.makedirs(self._csv_dir, exist_ok=True)
        ts = time.strftime("%Y-%m-%d %H:%M")
        rate = (self._failed / self._total * 100.0) if self._total > 0 else 0.0

        path = os.path.join(self._csv_dir, "invalid_goal_log.csv")
        existed = os.path.exists(path)

        detail_file = os.path.basename(getattr(self, '_detail_path', ''))

        with open(path, "a", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            if not existed or os.path.getsize(path) == 0:
                w.writerow([
                    "timestamp", "algo", "N_total", "N_invalid",
                    "invalid_rate_pct",
                    "timeout", "blocked", "rejected",
                    "server_unavailable", "unknown",
                    "detail_file",
                ])
            w.writerow([
                ts, self._algo_type, self._total, self._failed,
                f"{rate:.1f}",
                self._reason_counts["timeout"],
                self._reason_counts["blocked"],
                self._reason_counts["rejected"],
                self._reason_counts["server_unavailable"],
                self._reason_counts["unknown"],
                detail_file,
            ])
            f.flush()
        print(f"[GoalTracker] 汇总 → {path}")

    def _write_detail_csv(self):
        detail_dir = os.path.join(self._csv_dir, "invalid_goal_log")
        os.makedirs(detail_dir, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        path = os.path.join(detail_dir, f"{self._algo_type}_{ts}.csv")

        self._detail_path = path

        with open(path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow([
                "seq", "target_x", "target_y",
                "reached_x", "reached_y",
                "elapsed_s", "status", "note",
            ])
            for g in self._goals:
                w.writerow(list(g))
            f.flush()
        print(f"[GoalTracker] 详情 ({len(self._goals)} 点) → {path}")


# ── 模块级工具 ────────────────────────────────────────────────────

def _enrich(reason: str, labels: dict) -> str:
    """将 result_status:N 扩展为可读形式。"""
    if not reason.startswith("result_status:"):
        return reason
    try:
        code = int(reason.split(":", 1)[1])
        return f"result_status:{code}({labels.get(code, '?')})"
    except (ValueError, IndexError):
        return reason


def _fmt(val) -> str:
    """格式化坐标值，None → 'N/A'。"""
    if val is None:
        return "N/A"
    try:
        return f"{float(val):.3f}"
    except (TypeError, ValueError):
        return str(val)
