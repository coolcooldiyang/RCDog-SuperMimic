#!/usr/bin/env bash
# Fetch the manufacturer's dependency; only build and import, never move a motor.
set -euo pipefail
RCDOG_CONTROLLER=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
RCDOG_SDK="$RCDOG_CONTROLLER/third_party/unitree_actuator_sdk"
RCDOG_PYTHON=${RCDOG_PYTHON:-"$RCDOG_CONTROLLER/.venv/bin/python"}
if [[ ! -x "$RCDOG_PYTHON" ]]; then RCDOG_PYTHON=$(command -v python3); fi
RCDOG_SDK_REV=5b79a42d81cd69adac1367db79efdb972a898fd1
if [[ ! -d "$RCDOG_SDK/.git" ]]; then
    if [[ -e "$RCDOG_SDK" ]]; then echo "已有非Git SDK目录，未覆盖：$RCDOG_SDK" >&2; exit 1; fi
    mkdir -p "$(dirname -- "$RCDOG_SDK")"
    git init "$RCDOG_SDK"
    git -C "$RCDOG_SDK" fetch --depth 1 https://github.com/unitreerobotics/unitree_actuator_sdk.git "$RCDOG_SDK_REV"
    git -C "$RCDOG_SDK" checkout --detach FETCH_HEAD
fi
if [[ "$(git -C "$RCDOG_SDK" rev-parse HEAD)" != "$RCDOG_SDK_REV" ]]; then
    echo 'SDK版本与发布记录不一致，未修改已有SDK。' >&2; exit 1
fi
RCDOG_PATCH="$RCDOG_CONTROLLER/patches/unitree_sendrecv_release_gil.patch"
if git -C "$RCDOG_SDK" apply --check "$RCDOG_PATCH"; then
    git -C "$RCDOG_SDK" apply "$RCDOG_PATCH"
elif ! git -C "$RCDOG_SDK" apply --reverse --check "$RCDOG_PATCH"; then
    echo 'SDK绑定存在其他改动，补丁未应用。' >&2; exit 1
fi
cmake -S "$RCDOG_SDK" -B "$RCDOG_SDK/build" -DPYTHON_EXECUTABLE="$RCDOG_PYTHON" -DPYBIND11_FINDPYTHON=OFF
cmake --build "$RCDOG_SDK/build" --target unitree_actuator_sdk -j2
PYTHONPATH="$RCDOG_SDK/lib${PYTHONPATH:+:$PYTHONPATH}" LD_LIBRARY_PATH="$RCDOG_SDK/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
    "$RCDOG_PYTHON" -c 'import unitree_actuator_sdk as s; assert s.send_recv_releases_gil; print("SDK import OK; sendRecv releases GIL")'
