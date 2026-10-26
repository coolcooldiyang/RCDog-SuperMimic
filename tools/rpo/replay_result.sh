#!/usr/bin/env bash
set -euo pipefail
TASK_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
MOTION_NAME="${1:?usage: replay_result.sh run|backflip [eval|video]}"
REPLAY_MODE="${2:-eval}"
case "$MOTION_NAME" in run) RESULT_NAME=running; SECONDS_TO_RENDER=8 ;; backflip) RESULT_NAME=backflip; SECONDS_TO_RENDER=8.6 ;; *) exit 2 ;; esac
if [[ -x "$TASK_ROOT/.venv/bin/python" ]]; then
  PYTHON_BIN="${RPO_PYTHON:-$TASK_ROOT/.venv/bin/python}"
else
  PYTHON_BIN="${RPO_PYTHON:-python3}"
fi
cd "$TASK_ROOT"
export MUJOCO_GL="${MUJOCO_GL:-egl}"
CHECKPOINT="$TASK_ROOT/results/$RESULT_NAME/ckpt_002500.pt"
if [[ ! -f "$CHECKPOINT" ]]; then
  echo "Missing local checkpoint: $CHECKPOINT. See docs/REPRODUCE.md (weights are not redistributed)." >&2
  exit 1
fi
case "$REPLAY_MODE" in
  eval) exec "$PYTHON_BIN" -m unisupermimic.tasks.motion_tracking.rpo.supermimic.evaluate --ckpt "$CHECKPOINT" --backend mujoco ;;
  video) exec "$PYTHON_BIN" -m unisupermimic.tasks.motion_tracking.rpo.supermimic.render --ckpt "$CHECKPOINT" --backend mujoco --clip "${MOTION_NAME}_reference" --seconds "$SECONDS_TO_RENDER" --output "$TASK_ROOT/results/$RESULT_NAME/replayed.mp4" ;;
  *) exit 2 ;;
esac
