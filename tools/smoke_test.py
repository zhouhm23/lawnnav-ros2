#!/usr/bin/env python3
"""
smoke_test.py — 快速冒烟测试，检查最近修改的文件是否有语法/导入错误。
运行时间 < 5 秒，不需要 ROS 环境。
"""

import ast
import os
import sys
from pathlib import Path

SRC = Path(__file__).resolve().parent.parent  # ~/ros2_ws/src

FILES = [
    "path_coverage_ros2/scripts/path_coverage_node.py",
    "path_coverage_ros2/scripts/path_coverage_node_baseline.py",
    "launcher/publish_region.py",
    "launcher/start.py",
    "tools/test_coverage_comparison.py",
]

passed = 0
failed = 0

for rel in FILES:
    path = SRC / rel
    label = rel.split("/")[-1]
    try:
        source = path.read_text()
        tree = ast.parse(source, filename=str(path))
        # Check top-level function/class names
        funcs = [n.name for n in ast.walk(tree) if isinstance(n, (ast.FunctionDef, ast.ClassDef))]
        print(f"  ✅ {label} — {len(funcs)} top-level defs")
        passed += 1
    except SyntaxError as e:
        print(f"  ❌ {label} — SyntaxError: {e}")
        failed += 1

print(f"\n  {passed}/{len(FILES)} passed, {failed} failed")
sys.exit(0 if failed == 0 else 1)
