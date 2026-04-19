#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PORT="${1:-9000}"
OUTPUT_PREFIX="${2:-iphone_mono_coreml_orbslam3}"
MODEL_PATH="${3:-$ROOT_DIR/models/DepthAnythingV2SmallF16.mlpackage}"

export DYLD_LIBRARY_PATH="$ROOT_DIR/lib:$ROOT_DIR/Thirdparty/DBoW2/lib:$ROOT_DIR/Thirdparty/g2o/lib:$HOME/work/Pangolin/build${DYLD_LIBRARY_PATH:+:$DYLD_LIBRARY_PATH}"

"$ROOT_DIR/Examples/Monocular/map_iphone_coreml" \
  "$ROOT_DIR/Vocabulary/ORBvoc.txt" \
  "$MODEL_PATH" \
  "$PORT" \
  "$OUTPUT_PREFIX"
