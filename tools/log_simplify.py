#!/usr/bin/env python3
"""
log_simplify.py — ROS 2 日志精简器

遍历 logs/ros/*/launch.log，生成 launch_simplify.log：
  - 去掉时间戳、日期、非严重等级的 [xxx]
  - 默认仅保留 [ERROR] 和 [WARNING]，--info 同时保留 [INFO]
  - 连续行仅数字不同时折叠为最新行 + 次数标记

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

ANSI_RE = re.compile(r'\x1b\[[0-9;]*m')           # ANSI 转义码
EPOCH_RE = re.compile(r'^\d+\.?\d*\s+')            # 行首 epoch 时间戳
DATE_RE = re.compile(r'\d{4}-\d{2}-\d{2}\s+\d{2}:\d{2}:\d{2}')  # 日期时间
BRACKET_RE = re.compile(r'\[[^\]]*\]')              # 所有 [xxx]
SEVERITY_RE = re.compile(r'\[(ERROR|WARN|WARNING|INFO|FATAL|DEBUG)\]', re.IGNORECASE)
DIGITS_RE = re.compile(r'\d+')                      # 数字（用于骨架比较）
WHITESPACE_RE = re.compile(r'\s{2,}')               # 多余空白


def clean_line(raw: str) -> str | None:
    """清洗一行日志。返回 '[SEVERITY] message' 或 None（无有效等级）。"""
    # 1. 去 ANSI
    s = ANSI_RE.sub('', raw)
    # 2. 去行首 epoch 时间戳
    s = EPOCH_RE.sub('', s, count=1)
    # 3. 找严重等级
    m = SEVERITY_RE.search(s)
    if not m:
        return None
    sev = m.group(1).upper()
    if sev == 'WARNING':
        sev = 'WARN'
    # 4. 截取等级之后的内容
    idx = m.end()
    rest = s[idx:].strip()
    # 5. 去日期
    rest = DATE_RE.sub('', rest)
    # 6. 去剩余的 [xxx]（等级本身不在此范围内）
    rest = BRACKET_RE.sub(' ', rest)
    # 7. 去多余的冒号、空白
    rest = rest.lstrip(':').strip()
    rest = WHITESPACE_RE.sub(' ', rest)
    if not rest:
        return None
    return f'[{sev}] {rest}'


def skeleton(text: str) -> str:
    """摘掉所有数字，返回文字骨架用于去重比较。"""
    return DIGITS_RE.sub('#', text)


def process_one(log_path: str, keep_info: bool = False) -> int:
    """处理单个 launch.log，生成 launch_simplify.log。返回输出行数。"""
    out_path = os.path.join(os.path.dirname(log_path), 'launch_simplify.log')
    groups = []          # list of (cleaned_line, count)
    current_line = None
    current_skel = None
    current_count = 0

    with open(log_path, 'r', errors='replace') as f:
        for raw in f:
            cleaned = clean_line(raw)
            if cleaned is None:
                continue

            # 提取严重等级
            sev = cleaned[1:cleaned.index(']')]

            # 过滤
            if sev in ('ERROR', 'FATAL', 'WARN'):
                pass
            elif sev == 'INFO' and keep_info:
                pass
            else:
                continue

            skel = skeleton(cleaned)

            if skel == current_skel:
                # 同骨架：更新为最新行，递增计数
                current_line = cleaned
                current_count += 1
            else:
                # 不同骨架：先保存上一组，再开新组
                if current_line is not None:
                    groups.append((current_line, current_count))
                current_line = cleaned
                current_skel = skel
                current_count = 1

    # 保存最后一组
    if current_line is not None:
        groups.append((current_line, current_count))

    # 写入输出文件
    with open(out_path, 'w') as out:
        for line, cnt in groups:
            out.write(line + '\n')
            if cnt > 1:
                out.write(f'× {cnt}\n')
            out.write('\n')

    return len(groups)


def process_all(root: str, keep_info: bool = False):
    """遍历所有 logs/ros/*/launch.log 并处理。"""
    log_root = Path(root)
    if not log_root.is_dir():
        print(f'日志根目录不存在: {root}')
        return

    found = False
    for d in sorted(log_root.iterdir()):
        if not d.is_dir():
            continue
        launch_log = d / 'launch.log'
        if not launch_log.is_file():
            continue
        found = True
        n = process_one(str(launch_log), keep_info=keep_info)
        print(f'{d.name}/launch.log → {n} 类 → launch_simplify.log')

    if not found:
        print('未找到任何 launch.log。请先 source scripts/env_log.sh 并运行 ROS 程序。')


# ═══════════════════════════════════════════════════════════════════════════
# CLI
# ═══════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description='ROS 2 日志精简器')
    parser.add_argument('--dir', help='指定日志目录 (默认: 处理 logs/ros/ 下全部)')
    parser.add_argument('--info', action='store_true', help='同时保留 INFO 行')
    args = parser.parse_args()

    if args.dir:
        if os.path.isfile(os.path.join(args.dir, 'launch.log')):
            n = process_one(os.path.join(args.dir, 'launch.log'), keep_info=args.info)
            print(f'{n} 类 → launch_simplify.log')
        else:
            print(f'目录下无 launch.log: {args.dir}')
            sys.exit(1)
    else:
        LOG_ROOT = str(Path(__file__).resolve().parent.parent / 'logs' / 'ros')
        process_all(LOG_ROOT, keep_info=args.info)


if __name__ == '__main__':
    main()
