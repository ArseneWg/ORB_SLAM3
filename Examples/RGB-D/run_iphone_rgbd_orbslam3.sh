#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PORT="${1:-9000}"
OUTPUT_PREFIX="${2:-iphone_rgbd_orbslam3}"

export DYLD_LIBRARY_PATH="$ROOT_DIR/lib:$ROOT_DIR/Thirdparty/DBoW2/lib:$ROOT_DIR/Thirdparty/g2o/lib:$HOME/work/Pangolin/build${DYLD_LIBRARY_PATH:+:$DYLD_LIBRARY_PATH}"

"$ROOT_DIR/Examples/RGB-D/rgbd_iphone_stream" \
  "$ROOT_DIR/Vocabulary/ORBvoc.txt" \
  "$PORT" \
  "$OUTPUT_PREFIX"
