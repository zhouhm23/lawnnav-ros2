#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Compatibility entry: keep old bash command working.
exec python3 "$SCRIPT_DIR/start_path_coverage.py" "$@"
