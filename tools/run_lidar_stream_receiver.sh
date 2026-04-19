#!/bin/zsh
set -euo pipefail

PORT="${1:-9000}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "Starting LiDAR stream receiver on port ${PORT}"
python3 "${SCRIPT_DIR}/lidar_stream_receiver.py" --port "${PORT}"
