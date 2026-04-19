#!/bin/zsh
set -euo pipefail

cd "$(dirname "$0")"
xcodegen generate
open LiDARMapPreview.xcodeproj
