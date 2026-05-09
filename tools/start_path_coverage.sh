#!/usr/bin/env bash
# DEPRECATED — 请使用 launcher/start.sh
# 此文件保留仅为兼容旧指令，内部转发到新启动器。

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NEW_LAUNCHER="$SCRIPT_DIR/../launcher/start.sh"

if [ -f "$NEW_LAUNCHER" ]; then
    exec bash "$NEW_LAUNCHER" "$@"
else
    echo "[WARN] launcher/start.sh 不存在，回退到旧版 start_path_coverage.py"
    exec python3 "$SCRIPT_DIR/start_path_coverage.py" "$@"
fi