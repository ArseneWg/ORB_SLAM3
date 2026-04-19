#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/../.." && pwd)"
VOCAB_PATH="${1:-$ROOT_DIR/Vocabulary/ORBvoc.txt}"
SETTINGS_PATH="${2:-$ROOT_DIR/Examples/Monocular/MacBookAir_640x480.yaml}"
MODEL_PATH="${3:-$ROOT_DIR/models/DepthAnythingV2SmallF16.mlpackage}"
TRAJECTORY_PATH="${4:-$ROOT_DIR/KeyFrameTrajectory_iPhone.txt}"

detect_iphone_camera_index() {
  local devices
  devices="$(ffmpeg -f avfoundation -list_devices true -i "" 2>&1 || true)"
  printf '%s\n' "$devices" | awk '
    /AVFoundation video devices:/ { in_video=1; next }
    /AVFoundation audio devices:/ { in_video=0 }
    in_video && /\[[0-9]+\]/ && /iPhone/ && !/桌上视角/ {
      if (match($0, /\[[0-9]+\]/)) {
        print substr($0, RSTART + 1, RLENGTH - 2)
        exit
      }
    }
  '
}

IPHONE_CAMERA_INDEX="$(detect_iphone_camera_index)"

if [[ -z "$IPHONE_CAMERA_INDEX" ]]; then
  echo "No iPhone rear camera was detected via Continuity Camera." >&2
  echo "Make sure your iPhone is nearby, unlocked, and available as a camera on this Mac." >&2
  exit 1
fi

export DYLD_LIBRARY_PATH="$ROOT_DIR/lib:$ROOT_DIR/Thirdparty/DBoW2/lib:$ROOT_DIR/Thirdparty/g2o/lib:/Users/xy/work/Pangolin/build${DYLD_LIBRARY_PATH:+:$DYLD_LIBRARY_PATH}"

echo "Launching map demo with iPhone camera index: $IPHONE_CAMERA_INDEX"
echo "Vocabulary: $VOCAB_PATH"
echo "Settings:   $SETTINGS_PATH"
echo "Model:      $MODEL_PATH"

"$ROOT_DIR/Examples/Monocular/map_webcam_coreml" \
  "$VOCAB_PATH" \
  "$SETTINGS_PATH" \
  "$MODEL_PATH" \
  "$IPHONE_CAMERA_INDEX" \
  "$TRAJECTORY_PATH"
