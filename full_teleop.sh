#!/usr/bin/env bash
set -euo pipefail
TELEOP_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "$TELEOP_ROOT/run_python.sh" "$TELEOP_ROOT/scripts/run_upper_body_session.py" "$@"
