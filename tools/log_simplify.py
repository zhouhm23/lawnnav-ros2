#!/usr/bin/env python3
"""
log_simplify.py — ROS 2 日志精简器

以每个 launch 时间戳目录为主体，收集同次运行的所有节点日志
（PID 在 launch PID ±3000 范围内），合并生成 launch_simplify.log。

用法:
    python3 tools/log_simplify.py                 # 处理所有日志目录
    python3 tools/log_simplify.py --info           # 同时保留 INFO
    python3 tools/log_simplify.py --dir DIR        # 只处理指定目录
"""

import argparse
import os
import re
import sys
from pathlib import Path

# ═══════════════════════════════════════════════════════════════════════════
# 正则
# ═══════════════════════════════════════════════════════════════════════════

ANSI_RE = re.compile(r'\x1b\[[0-9;]*m')
EPOCH_RE = re.compile(r'^\d+\.?\d*\s+')
DATE_RE = re.compile(r'\d{4}-\d{2}-\d{2}\s+\d{2}:\d{2}:\d{2}')
BRACKET_RE = re.compile(r'\[[^\]]*\]')
SEVERITY_RE = re.compile(r'\[(ERROR|WARN|WARNING|INFO|FATAL|DEBUG)\]', re.IGNORECASE)
DIGITS_RE = re.compile(r'\d+')
WHITESPACE_RE = re.compile(r'\s{2,}')

# 日志文件名格式: {node_name}_{PID}_{timestamp}.log
FLAT_LOG_RE = re.compile(r'^(.+)_(\d+)_(\d+)\.log$')
# 时间戳目录名提取 PID: ...-raspberrypi-909026
DIR_PID_RE = re.compile(r'-(\d+)$')

PID_WINDOW = 3000


def clean_line(raw: str) -> str | None:
    """清洗一行日志。返回 '[SEVERITY] message' 或 None。"""
    s = ANSI_RE.sub('', raw)
    s = EPOCH_RE.sub('', s, count=1)
    m = SEVERITY_RE.search(s)
    if not m:
        return None
    sev = m.group(1).upper()
    if sev == 'WARNING':
        sev = 'WARN'
    rest = s[m.end():].strip()
    rest = DATE_RE.sub('', rest)
    rest = BRACKET_RE.sub(' ', rest)
    rest = rest.lstrip(':').strip()
    rest = WHITESPACE_RE.sub(' ', rest)
    if not rest:
        return None
    return f'[{sev}] {rest}'


def skeleton(text: str) -> str:
    return DIGITS_RE.sub('#', text)


def extract_pid_from_logfile(fname: str) -> int | None:
    """从扁平日志文件名提取 PID，如 'ekf_node_817591_1779521038578.log' → 817591。"""
    m = FLAT_LOG_RE.match(fname)
    if m:
        return int(m.group(2))
    return None


def extract_pid_from_dirname(dirname: str) -> int | None:
    """从时间戳目录名提取 PID，如 '2026-05-23-07-45-03-raspberrypi-909026' → 909026。"""
    m = DIR_PID_RE.search(dirname)
    if m:
        return int(m.group(1))
    return None


def collect_session_logs(log_root: str, launch_pid: int, launch_dir: str):
    """收集同一次运行的所有日志文件，按 PID 排序返回 (pid, filepath) 列表。"""
    files = []

    # 1. launch.log（放在最前面，PID=0 排最前）
    launch_log = os.path.join(launch_dir, 'launch.log')
    if os.path.isfile(launch_log):
        files.append((0, launch_log))

    # 2. logs/ros/ 下的扁平日志，PID 在 [launch_pid, launch_pid+PID_WINDOW] 范围
    lo = launch_pid
    hi = launch_pid + PID_WINDOW
    try:
        for fname in os.listdir(log_root):
            fpath = os.path.join(log_root, fname)
            if not os.path.isfile(fpath):
                continue
            if not fname.endswith('.log'):
                continue
            pid = extract_pid_from_logfile(fname)
            if pid is not None and lo <= pid <= hi:
                files.append((pid, fpath))
    except OSError:
        pass

    # 按 PID 排序
    files.sort(key=lambda x: x[0])
    return [fp for _, fp in files]


def process_session(log_root: str, launch_dir: str, keep_info: bool = False) -> int:
    """处理一次运行的完整日志，生成 launch_simplify.log。"""
    launch_pid = extract_pid_from_dirname(os.path.basename(launch_dir))
    if launch_pid is None:
        print(f'  无法从目录名提取 PID: {launch_dir}')
        return 0

    log_files = collect_session_logs(log_root, launch_pid, launch_dir)
    if not log_files:
        return 0

    out_path = os.path.join(launch_dir, 'launch_simplify.log')
    groups = []
    cur_line = None
    cur_skel = None
    cur_cnt = 0

    for fpath in log_files:
        try:
            with open(fpath, 'r', errors='replace') as f:
                for raw in f:
                    c = clean_line(raw)
                    if c is None:
                        continue
                    sev = c[1:c.index(']')]
                    if sev in ('ERROR', 'FATAL', 'WARN'):
                        pass
                    elif sev == 'INFO' and keep_info:
                        pass
                    else:
                        continue
                    sk = skeleton(c)
                    if sk == cur_skel:
                        cur_line = c
                        cur_cnt += 1
                    else:
                        if cur_line is not None:
                            groups.append((cur_line, cur_cnt))
                        cur_line = c
                        cur_skel = sk
                        cur_cnt = 1
        except OSError:
            pass

    if cur_line is not None:
        groups.append((cur_line, cur_cnt))

    # 输出时首行加来源文件数
    with open(out_path, 'w') as out:
        out.write(f'# 来源: {len(log_files)} 个日志文件 (launch PID={launch_pid}, +{PID_WINDOW})\n')
        for line, cnt in groups:
            out.write(line + '\n')
            if cnt > 1:
                out.write(f'× {cnt}\n')
            out.write('\n')

    return len(groups)


def process_all(root: str, keep_info: bool = False):
    """遍历所有 logs/ros/*/launch.log 时间戳目录并处理。"""
    log_root = Path(root)
    if not log_root.is_dir():
        print(f'日志根目录不存在: {root}')
        return

    # 收集所有时间戳目录（含 launch.log 的）
    dirs = []
    for d in sorted(log_root.iterdir()):
        if not d.is_dir():
            continue
        if not (d / 'launch.log').is_file():
            continue
        dirs.append(d)

    if not dirs:
        print('未找到任何含 launch.log 的目录。')
        return

    for d in dirs:
        n = process_session(root, str(d), keep_info=keep_info)
        print(f'{d.name} → {n} 类 ({len(list(d.iterdir()))} files) → launch_simplify.log')


def main():
    parser = argparse.ArgumentParser(description='ROS 2 日志精简器')
    parser.add_argument('--dir', help='指定日志时间戳目录')
    parser.add_argument('--info', action='store_true', help='同时保留 INFO 行')
    args = parser.parse_args()

    LOG_ROOT = str(Path(__file__).resolve().parent.parent / 'logs' / 'ros')

    if args.dir:
        n = process_session(LOG_ROOT, args.dir, keep_info=args.info)
        print(f'{n} 类 → launch_simplify.log')
    else:
        process_all(LOG_ROOT, keep_info=args.info)


if __name__ == '__main__':
    main()
