#!/usr/bin/env bash

set -Eeuo pipefail

ROOT="$HOME/star_agibot"

WHEELHOUSE="$ROOT/runtime/wheelhouse_aarch64"
RUNTIME_ROOT="$ROOT/runtime/aarch64_py310"
SITE_PACKAGES="$RUNTIME_ROOT/site-packages"

DOWNLOAD_PYTHON="$ROOT/asr_offline_lab/venv/bin/python"

SHERPA_VERSION="1.13.4"

echo "============================================================"
echo "构建 ARM64 Python 3.10 自包含运行目录"
echo "============================================================"
echo "项目目录：$ROOT"
echo "wheel目录：$WHEELHOUSE"
echo "目标目录：$SITE_PACKAGES"
echo

if [[ ! -d "$ROOT" ]]; then
    echo "错误：项目目录不存在：$ROOT"
    exit 1
fi

if [[ ! -x "$DOWNLOAD_PYTHON" ]]; then
    echo "错误：下载工具环境不存在："
    echo "$DOWNLOAD_PYTHON"
    exit 1
fi

mkdir -p "$WHEELHOUSE"

echo "========== 第1步：补齐 ARM64 wheel 及其依赖 =========="

"$DOWNLOAD_PYTHON" -m pip download \
    "sherpa-onnx==$SHERPA_VERSION" \
    --dest "$WHEELHOUSE" \
    --find-links "$WHEELHOUSE" \
    --only-binary=:all: \
    --platform manylinux2014_aarch64 \
    --python-version 310 \
    --implementation cp \
    --abi cp310

echo
echo "========== wheel文件 =========="

find "$WHEELHOUSE" \
    -maxdepth 1 \
    -type f \
    -name '*.whl' \
    -printf '%f\n' \
    | sort

echo
echo "========== 检查错误平台wheel =========="

bad_wheels="$(
    find "$WHEELHOUSE" \
        -maxdepth 1 \
        -type f \
        -name '*.whl' \
        -printf '%f\n' \
        | grep -E \
          'x86_64|i686|win32|win_amd64|macosx|cp311|cp312|cp313|cp314' \
        || true
)"

if [[ -n "$bad_wheels" ]]; then
    echo "错误：发现不适用于ARM64 Python 3.10的wheel："
    echo "$bad_wheels"
    exit 1
fi

main_wheel_count="$(
    find "$WHEELHOUSE" \
        -maxdepth 1 \
        -type f \
        -name \
        "sherpa_onnx-${SHERPA_VERSION}-cp310-cp310-*aarch64*.whl" \
        | wc -l
)"

if [[ "$main_wheel_count" -ne 1 ]]; then
    echo "错误：应当存在1个ARM64 CPython 3.10主wheel，"
    echo "实际数量：$main_wheel_count"
    exit 1
fi

echo "wheel平台检查通过"

echo
echo "========== 第2步：读取主wheel实际依赖 =========="

"$DOWNLOAD_PYTHON" - <<'PY'
from pathlib import Path
from zipfile import ZipFile

root = Path.home() / "star_agibot"
wheelhouse = root / "runtime" / "wheelhouse_aarch64"

wheels = sorted(
    wheelhouse.glob(
        "sherpa_onnx-1.13.4-cp310-cp310-*aarch64*.whl"
    )
)

if len(wheels) != 1:
    raise SystemExit(
        f"主wheel数量异常：{len(wheels)}"
    )

wheel = wheels[0]

with ZipFile(wheel) as archive:
    metadata_files = [
        item
        for item in archive.namelist()
        if item.endswith(".dist-info/METADATA")
    ]

    if len(metadata_files) != 1:
        raise SystemExit(
            f"METADATA数量异常：{len(metadata_files)}"
        )

    metadata = archive.read(
        metadata_files[0]
    ).decode(
        "utf-8",
        errors="replace",
    )

requirements = [
    line
    for line in metadata.splitlines()
    if line.startswith("Requires-Dist:")
]

print("主wheel：", wheel.name)

if requirements:
    print("声明的依赖：")

    for requirement in requirements:
        print("  ", requirement)
else:
    print("该wheel没有声明额外依赖")
PY

echo
echo "========== 第3步：安装到自包含目录 =========="

rm -rf "$RUNTIME_ROOT"
mkdir -p "$SITE_PACKAGES"

"$DOWNLOAD_PYTHON" -m pip install \
    "sherpa-onnx==$SHERPA_VERSION" \
    --target "$SITE_PACKAGES" \
    --no-index \
    --find-links "$WHEELHOUSE" \
    --only-binary=:all: \
    --platform manylinux2014_aarch64 \
    --python-version 310 \
    --implementation cp \
    --abi cp310 \
    --no-compile

echo
echo "========== 已安装的Python发行包 =========="

find "$SITE_PACKAGES" \
    -maxdepth 1 \
    -type d \
    -name '*.dist-info' \
    -printf '%f\n' \
    | sort

echo
echo "========== 第4步：检查二进制架构 =========="

shared_object_count=0
bad_architecture=0

while IFS= read -r -d '' shared_object; do
    shared_object_count=$((shared_object_count + 1))

    description="$(
        file -Lb "$shared_object"
    )"

    relative_path="${shared_object#"$SITE_PACKAGES/"}"

    echo "$relative_path"
    echo "  $description"

    if grep -Eq \
        'x86-64|Intel 80386' \
        <<< "$description"
    then
        echo "  错误：发现x86二进制文件"
        bad_architecture=1
    fi

done < <(
    find "$SITE_PACKAGES" \
        -type f \
        \( \
            -name '*.so' \
            -o -name '*.so.*' \
        \) \
        -print0
)

if [[ "$shared_object_count" -eq 0 ]]; then
    echo "错误：没有找到任何共享库"
    exit 1
fi

if [[ "$bad_architecture" -ne 0 ]]; then
    echo "错误：自包含目录中混入了x86库"
    exit 1
fi

if ! find "$SITE_PACKAGES" \
    -type f \
    -name '_sherpa_onnx*.so' \
    -print \
    -quit \
    | grep -q .
then
    echo "错误：没有找到_sherpa_onnx Python扩展"
    exit 1
fi

echo
echo "ARM64静态架构检查通过"

echo
echo "========== 第5步：生成运行环境加载脚本 =========="

cat > "$ROOT/scripts/activate_robot_runtime.sh" <<'ACTIVATE'
#!/usr/bin/env bash

# 本文件必须使用source加载：
#
# source ~/star_agibot/scripts/activate_robot_runtime.sh

STAR_AGIBOT_ROOT="$(
    cd "$(
        dirname "${BASH_SOURCE[0]}"
    )/.." \
    && pwd
)"

ARM64_SITE_PACKAGES="$STAR_AGIBOT_ROOT/runtime/aarch64_py310/site-packages"

if [[ ! -d "$ARM64_SITE_PACKAGES" ]]; then
    echo "错误：ARM64运行目录不存在："
    echo "$ARM64_SITE_PACKAGES"
    return 1 2>/dev/null || exit 1
fi

export STAR_AGIBOT_ROOT

export PYTHONPATH="$ARM64_SITE_PACKAGES${PYTHONPATH:+:$PYTHONPATH}"

# 避免比赛机器人加载用户目录中不受控的Python包。
export PYTHONNOUSERSITE=1

runtime_library_path="$ARM64_SITE_PACKAGES"

while IFS= read -r library_directory; do
    case ":$runtime_library_path:" in
        *":$library_directory:"*)
            ;;
        *)
            runtime_library_path="$runtime_library_path:$library_directory"
            ;;
    esac

done < <(
    find "$ARM64_SITE_PACKAGES" \
        -type f \
        \( \
            -name '*.so' \
            -o -name '*.so.*' \
        \) \
        -printf '%h\n' \
        | sort -u
)

export LD_LIBRARY_PATH="$runtime_library_path${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
ACTIVATE

chmod +x \
    "$ROOT/scripts/activate_robot_runtime.sh"

echo "已生成："
echo "$ROOT/scripts/activate_robot_runtime.sh"

echo
echo "========== 第6步：生成完整性校验文件 =========="

(
    cd "$RUNTIME_ROOT"

    find site-packages \
        -type f \
        -print0 \
        | LC_ALL=C sort -z \
        | xargs -0 sha256sum \
        > manifest.sha256
)

file_count="$(
    wc -l \
    < "$RUNTIME_ROOT/manifest.sha256"
)"

echo "已记录文件数量：$file_count"

echo
echo "========== 构建完成 =========="

du -sh "$RUNTIME_ROOT"
du -sh "$WHEELHOUSE"

echo
echo "ARM64自包含运行目录："
echo "$RUNTIME_ROOT"

echo
echo "机器人加载命令："
echo "source ~/star_agibot/scripts/activate_robot_runtime.sh"
