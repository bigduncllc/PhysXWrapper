#!/usr/bin/env bash
set -euo pipefail

BUILD_DIR="linux-release-build"

if [ -d "$BUILD_DIR" ]; then
  rm -rf "$BUILD_DIR"
fi

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

cmake .. -DCMAKE_BUILD_TYPE=Release

cmake --build . -- -j"$(nproc)"
