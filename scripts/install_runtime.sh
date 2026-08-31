#!/usr/bin/env bash

set -euo pipefail

ROOT="$HOME/star_agibot"
WHEELHOUSE="$ROOT/runtime/wheelhouse_aarch64"
TARGET="$ROOT/runtime/python"

mkdir -p "$TARGET"

python3 -m pip install \
  --no-index \
  --find-links "$WHEELHOUSE" \
  --target "$TARGET" \
  --upgrade \
  --no-deps \
  sherpa-onnx==1.13.4

echo
echo "运行库安装完成：$TARGET"
