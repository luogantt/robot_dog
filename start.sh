#!/usr/bin/env bash
# ============================================================
#  Robot Dog SDK launcher (Linux - for the robot host)
#
#      ./start.sh                menu
#      ./start.sh sim            local simulation + web UI
#      ./start.sh observe        read-only, robot will not move
#      ./start.sh control        live control (careful)
#      ./start.sh status         read-only status query
#      ./start.sh check          environment check
#
#  All messages live in start.py, which is cross-platform.
# ============================================================
set -u
cd "$(dirname "$0")"

export PYTHONIOENCODING=utf-8

PY=""
for c in python3 python; do
    if command -v "$c" >/dev/null 2>&1; then PY="$c"; break; fi
done

if [ -z "$PY" ]; then
    echo "[ERROR] python3 not found. Install Python 3.10+ first."
    exit 1
fi

exec "$PY" start.py "$@"
