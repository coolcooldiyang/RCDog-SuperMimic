"""SuperMimic networks can be loaded without the training runtime."""
__all__ = ["SuperMimicConfig", "SuperMimicLearner"]
def __getattr__(name):
    if name in __all__:
        from . import learner
        return getattr(learner, name)
    raise AttributeError(name)
