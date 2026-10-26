# RPO adaptation, 2026-10-26; derived from Uni-SuperMimic where applicable.
# RPO extension / combined software: GPL-3.0-only. See LICENSE and THIRD_PARTY.md.
"""Register the RPO SuperMimic task with UniLab."""

from unilab.base import registry
from unilab.envs import ManagerBasedRlEnvCfg

from unisupermimic.tasks.motion_tracking.rpo.supermimic.env import (
    SUPPORTED_BACKENDS,
    make_supermimic_env,
)

TASK_NAME = "RPOSuperMimic"

registry.register_env_config(TASK_NAME, ManagerBasedRlEnvCfg)
for backend in SUPPORTED_BACKENDS:
    registry.register_env(TASK_NAME, make_supermimic_env, sim_backend=backend)

__all__ = ["TASK_NAME"]
