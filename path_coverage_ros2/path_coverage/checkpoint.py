"""
checkpoint.py — 覆盖任务断点续跑（共享模块，创新版与 baseline 共用）。

用法:
    from path_coverage.checkpoint import (
        save_checkpoint, load_checkpoint, clear_checkpoint,
        register_signal_handlers, CHECKPOINT_FILE,
    )

设计原则:
    - segment 粒度保存（不是 cell 级别），续跑时最多重复 1 个 segment
    - 仅异常退出（SIGINT/SIGTERM/崩溃）时保留 checkpoint
    - 正常完成或 q 放弃时清除 checkpoint
    - polygon_hash 校验区域是否变更
    - 原子写入（临时文件 + rename）防崩溃写坏
"""

import hashlib
import json
import os
import signal
import tempfile
from typing import Any, Dict, Optional

CHECKPOINT_FILE = "/tmp/path_coverage_checkpoint.json"

# ── 内部状态 ──────────────────────────────────────────────────────
_should_clear_on_exit = True  # 正常完成/q 退出时设为 True


def _hash_polygon(polygon) -> str:
    """Compute a stable hash of polygon vertices for region-change detection."""
    try:
        if hasattr(polygon, "exterior"):
            coords = list(polygon.exterior.coords)
        else:
            coords = list(polygon)
        raw = json.dumps(coords, sort_keys=True).encode("utf-8")
        return hashlib.md5(raw).hexdigest()[:12]
    except Exception:
        return "unknown"


def save_checkpoint(
    cell_idx: int,
    segment_idx: int,
    total_cells: int,
    polygon=None,
    region_file: str = "",
) -> None:
    """Save current progress to checkpoint file (atomic write)."""
    data: Dict[str, Any] = {
        "cell_idx": cell_idx,
        "segment_idx": segment_idx,
        "total_cells": total_cells,
        "polygon_hash": _hash_polygon(polygon) if polygon is not None else "",
        "region_file": region_file,
    }
    # Atomic write: temp file + rename
    try:
        fd, tmp_path = tempfile.mkstemp(
            suffix=".json", prefix="checkpoint_", dir="/tmp"
        )
        with os.fdopen(fd, "w") as f:
            json.dump(data, f)
        os.rename(tmp_path, CHECKPOINT_FILE)
    except OSError:
        pass  # 非关键，静默失败


def load_checkpoint() -> Optional[Dict[str, Any]]:
    """Load checkpoint if exists, else None."""
    if not os.path.exists(CHECKPOINT_FILE):
        return None
    try:
        with open(CHECKPOINT_FILE, "r") as f:
            return json.load(f)
    except (json.JSONDecodeError, OSError):
        return None


def clear_checkpoint() -> None:
    """Remove checkpoint file (normal completion or abandon)."""
    global _should_clear_on_exit
    _should_clear_on_exit = True
    try:
        os.remove(CHECKPOINT_FILE)
    except OSError:
        pass


def _checkpoint_on_signal(signum, frame):
    """Signal handler: mark that we should NOT clear checkpoint on exit."""
    global _should_clear_on_exit
    _should_clear_on_exit = False
    # Re-raise KeyboardInterrupt for normal SIGINT handling
    raise KeyboardInterrupt(f"Received signal {signum}")


def register_signal_handlers():
    """Register SIGINT/SIGTERM handlers that preserve checkpoint.

    Call this once at node startup.
    """
    signal.signal(signal.SIGINT, _checkpoint_on_signal)
    signal.signal(signal.SIGTERM, _checkpoint_on_signal)


def mark_normal_exit():
    """Call when coverage completes normally or user quits.

    Returns True if checkpoint should be cleared.
    """
    global _should_clear_on_exit
    _should_clear_on_exit = True
    return True


def should_clear() -> bool:
    """Check whether checkpoint should be cleared on exit."""
    return _should_clear_on_exit
