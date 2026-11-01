#!/usr/bin/env bash
set -euo pipefail
TELEOP_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
TELEOP_ENV_PATH="${TELEOP_ENV:-$TELEOP_ROOT/env}"
if [[ ! -x "$TELEOP_ENV_PATH/bin/python" ]]; then
  echo 'Python environment missing. Run ./rebuild_environment.sh or set TELEOP_ENV to an existing prefix.' >&2
  exit 2
fi
export PYTHONNOUSERSITE=1
export PYTHONUNBUFFERED=1
export PYTHONPATH="$TELEOP_ROOT/scripts:$TELEOP_ROOT/repos/xr_teleoperate:$TELEOP_ROOT/repos/xr_teleoperate/teleop"
export MPLCONFIGDIR="$TELEOP_ROOT/.cache/matplotlib"
export PIP_CACHE_DIR="$TELEOP_ROOT/.cache/pip"
mkdir -p "$MPLCONFIGDIR" "$PIP_CACHE_DIR"
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-1}"
export OPENBLAS_NUM_THREADS=1
export MKL_NUM_THREADS=1
export XR_TELEOP_CERT="$TELEOP_ROOT/certs/cert.pem"
export XR_TELEOP_KEY="$TELEOP_ROOT/certs/key.pem"
cd "$TELEOP_ROOT"
exec "$TELEOP_ENV_PATH/bin/python" "$@"
