#!/usr/bin/env bash
# Compile only. Never invoke the generated driver or its example executables.
set -euo pipefail
TELEOP_SOURCE=""
TELEOP_SDK_PREFIX=/usr/local
TELEOP_JOBS=2
while (($#)); do
  case "$1" in
    --source) TELEOP_SOURCE="$2"; shift 2 ;;
    --sdk-prefix) TELEOP_SDK_PREFIX="$2"; shift 2 ;;
    --jobs) TELEOP_JOBS="$2"; shift 2 ;;
    -h|--help)
      echo 'Usage: bash build_robot_driver.sh --source /path/DFX_inspire_service [--sdk-prefix /usr/local] [--jobs 2]'
      echo 'Requires installed Unitree SDK2 C++ with its matching CycloneDDS C/C++ libraries, Boost program_options, spdlog and Eigen.'
      echo 'Builds inspire_g1 only; does not run it or install a system service.'
      exit 0 ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done
[[ -n "$TELEOP_SOURCE" && -f "$TELEOP_SOURCE/CMakeLists.txt" ]] || { echo 'Provide the patched DFX source with --source.' >&2; exit 2; }
[[ "$TELEOP_JOBS" =~ ^[1-9][0-9]*$ ]] || { echo '--jobs must be positive.' >&2; exit 2; }
[[ "$TELEOP_SDK_PREFIX" = /* && "$TELEOP_SDK_PREFIX" =~ ^[A-Za-z0-9_./-]+$ ]] || { echo '--sdk-prefix must be a simple absolute path without spaces or shell characters.' >&2; exit 2; }
command -v cmake >/dev/null || { echo 'Missing cmake (install build-essential cmake).' >&2; exit 2; }
[[ -f "$TELEOP_SDK_PREFIX/include/unitree/robot/channel/channel_publisher.hpp" ]] || {
  echo "Unitree SDK2 C++ headers missing from $TELEOP_SDK_PREFIX/include." >&2
  echo 'Install the robot-compatible SDK2 C++ distribution, including matching ddsc/ddscxx, before building.' >&2
  exit 2
}
[[ -f "$TELEOP_SDK_PREFIX/include/ddscxx/dds/dds.hpp" ]] || { echo 'Matching CycloneDDS C++ headers are missing.' >&2; exit 2; }
# These checks prevent accidentally building the original short-read serial implementation.
python3 - "$TELEOP_SOURCE" <<'PY'
from pathlib import Path
import sys
p=Path(sys.argv[1])
required={'include/inspire.h': ['milliseconds(40)', 'recv_some_until'],
          'include/SerialPort.h': ['Deadline', 'recv_some_until'],
          'inspire_g1.cpp': ['quiet_NaN', 'SetPosition(qcmd.block'],
          'CMakeLists.txt': ['target_link_libraries(inspire_g1 spdlog::spdlog)']}
for name, markers in required.items():
    text=(p/name).read_text()
    if not all(marker in text for marker in markers):
        raise SystemExit('Complete DFX serial patch missing in '+name)
PY
cmake -S "$TELEOP_SOURCE" -B "$TELEOP_SOURCE/build" -DCMAKE_BUILD_TYPE=Release \
  "-DCMAKE_PREFIX_PATH=$TELEOP_SDK_PREFIX" \
  "-DCMAKE_CXX_FLAGS=-I$TELEOP_SDK_PREFIX/include -I$TELEOP_SDK_PREFIX/include/ddscxx -I$TELEOP_SDK_PREFIX/include/iceoryx/v2.0.2" \
  "-DCMAKE_EXE_LINKER_FLAGS=-L$TELEOP_SDK_PREFIX/lib -Wl,-rpath,$TELEOP_SDK_PREFIX/lib"
cmake --build "$TELEOP_SOURCE/build" --target inspire_g1 --parallel "$TELEOP_JOBS"
echo "Built only: $TELEOP_SOURCE/build/inspire_g1"
echo 'No driver, DDS participant, serial device, or startup service was started.'
