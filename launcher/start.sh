#!/usr/bin/env bash
# 割草机器人系统一键启动（Shell 兼容包装）
# 实际逻辑在 launcher/start.py

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "$SCRIPT_DIR/start.py" "$@"
