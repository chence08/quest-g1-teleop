#!/usr/bin/env bash
# Workstation only. Downloads/builds dependencies; starts no robot or XR process.
set -euo pipefail
TELEOP_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
TELEOP_ENV_PATH="${TELEOP_ENV:-$TELEOP_ROOT/env}"
TELEOP_EXACT=0
while (($#)); do
  case "$1" in
    --prefix) TELEOP_ENV_PATH="$2"; shift 2 ;;
    --exact-lock) TELEOP_EXACT=1; shift ;;
    -h|--help)
      echo 'Usage: ./rebuild_environment.sh [--prefix /absolute/path] [--exact-lock]'
      echo 'Default: portable x86-64 solve with pinned core packages; pip uses recorded constraints.'
      echo '--exact-lock: original Linux x86-64/Zen4 Conda artifacts; requires a compatible CPU.'
      exit 0 ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done
[[ "$(uname -s)" == Linux && "$(uname -m)" == x86_64 ]] || { echo 'Workstation build requires Linux x86_64.' >&2; exit 2; }
[[ "$TELEOP_ENV_PATH" = /* ]] || { echo '--prefix/TELEOP_ENV must be absolute.' >&2; exit 2; }
[[ ! -e "$TELEOP_ENV_PATH" ]] || { echo "Destination exists; choose a fresh prefix: $TELEOP_ENV_PATH" >&2; exit 2; }
command -v conda >/dev/null || { echo 'Install Miniforge/Conda and put conda on PATH first.' >&2; exit 2; }
python3 "$TELEOP_ROOT/scripts/fetch_upstream.py"
if ((TELEOP_EXACT)); then
  conda create --prefix "$TELEOP_ENV_PATH" --file "$TELEOP_ROOT/conda-linux-64.explicit.txt" -y
else
  CONDA_OVERRIDE_ARCHSPEC=x86_64 conda create --prefix "$TELEOP_ENV_PATH" --override-channels -c conda-forge \
    'python=3.10.22' 'pinocchio=3.1.0' 'numpy=1.26.4' 'casadi=3.6.7' pip -y
fi
"$TELEOP_ENV_PATH/bin/python" -m pip install 'torch==2.3.0+cpu' --index-url https://download.pytorch.org/whl/cpu
"$TELEOP_ENV_PATH/bin/python" -m pip install -c "$TELEOP_ROOT/pip-versions.lock" \
  -r "$TELEOP_ROOT/repos/xr_teleoperate/requirements.txt" \
  -e "$TELEOP_ROOT/repos/unitree_sdk2_python" \
  -e "$TELEOP_ROOT/repos/xr_teleoperate/teleop/televuer" \
  -e "$TELEOP_ROOT/repos/xr_teleoperate/teleop/teleimager[server]" \
  -e "$TELEOP_ROOT/repos/xr_teleoperate/teleop/robot_control/dex-retargeting" \
  -e "$TELEOP_ROOT/repos/inspire_hand_ws/inspire_hand_sdk"
"$TELEOP_ENV_PATH/bin/python" -m pip check
"$TELEOP_ENV_PATH/bin/python" -c 'import numpy, pinocchio, casadi, torch, vuer; print("Core imports passed", numpy.__version__, pinocchio.__version__, casadi.__version__, torch.__version__)'
echo "Environment built: $TELEOP_ENV_PATH"
echo 'For a nondefault prefix, export TELEOP_ENV to that absolute path before using run_python.sh.'
