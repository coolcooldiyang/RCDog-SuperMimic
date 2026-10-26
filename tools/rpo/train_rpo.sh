#!/usr/bin/env bash
set -euo pipefail
TASK_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
MOTION_NAME="${1:?usage: train_rpo.sh run|backflip [iterations]}"
TRAIN_ITERATIONS="${2:-12000}"
PHYSICS_MODE="${3:-coupled}"
case "$MOTION_NAME" in run|backflip) ;; *) exit 2 ;; esac
case "$PHYSICS_MODE" in coupled|independent) ;; *) exit 2 ;; esac
cd "$TASK_ROOT"
export PYTHONUNBUFFERED=1 OMP_NUM_THREADS=4 MUJOCO_GL=egl
export TORCHINDUCTOR_CACHE_DIR="$TASK_ROOT/torchinductor_cache"
export TRITON_CACHE_DIR="$TASK_ROOT/triton_cache"
MODEL_ARGS=()
RUN_VERSION=v1
if [[ "$PHYSICS_MODE" == coupled ]]; then
  RUN_VERSION=v2
  MODEL_ARGS+=(env.scene.model_file=src/unisupermimic/assets/robots/rpo/scene_supermimic_coupled.xml)
  MODEL_ARGS+=(env.commands.motion.mjcf_path=src/unisupermimic/assets/robots/rpo/supermimic_coupled.xml)
  if [[ -f "$TASK_ROOT/initializers/rpo_${MOTION_NAME}_v1_last.pt" ]]; then
    MODEL_ARGS+=("+training.init_checkpoint=$TASK_ROOT/initializers/rpo_${MOTION_NAME}_v1_last.pt")
  fi
fi
exec "${RPO_TRAIN_BIN:-$TASK_ROOT/.venv/bin/unisupermimic-train}" \
  task=rpo_super_mimic/mjwarp \
  "${MODEL_ARGS[@]}" \
  "env.commands.motion.motion_files=[src/unisupermimic/assets/motions/rpo/${MOTION_NAME}_reference.csv]" \
  algo.num_envs=1024 "algo.iterations=$TRAIN_ITERATIONS" \
  'algo.wm_hidden=[1024,1024,1024]' \
  'algo.policy_hidden=[1024,512,256,128]' \
  algo.wm_batch=1024 algo.pi_batch=1024 algo.wm_batches=10 \
  algo.wm_only_iters=500 algo.ckpt_every=500 algo.log_every=25 \
  "training.log_dir=runs/rpo_${MOTION_NAME}_${RUN_VERSION}"
