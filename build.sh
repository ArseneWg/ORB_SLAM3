#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if command -v sysctl >/dev/null 2>&1; then
  BUILD_JOBS="$(sysctl -n hw.ncpu)"
elif command -v nproc >/dev/null 2>&1; then
  BUILD_JOBS="$(nproc)"
else
  BUILD_JOBS=4
fi

echo "Uncompress vocabulary ..."

cd "${ROOT_DIR}/Vocabulary"
if [ ! -f ORBvoc.txt ]; then
  tar -xf ORBvoc.txt.tar.gz
fi
cd "${ROOT_DIR}"

echo "Configuring and building ORB_SLAM3 ..."

cmake -S "${ROOT_DIR}" -B "${ROOT_DIR}/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "${ROOT_DIR}/build" -j"${BUILD_JOBS}"
