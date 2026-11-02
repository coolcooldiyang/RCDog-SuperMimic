"""RCDog SuperMimic; simulator dependencies load only when needed."""
__all__ = ["SuperMimicManagerBasedRlEnv", "make_supermimic_env"]
def __getattr__(name):
    if name in __all__:
        from . import env
        return getattr(env, name)
    raise AttributeError(name)
