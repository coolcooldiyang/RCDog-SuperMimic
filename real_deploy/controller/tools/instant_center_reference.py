"""Runtime adapter; retain the checksummed exported gait and reference source."""
import numpy as np


def immediate_center_reference(reference_class):
    class ImmediateCenterReference(reference_class):
        immediate_zero=True

        def prepare_command(self,requested):
            if self.immediate_zero and not np.any(np.asarray(requested,float)):
                self.command[:]=0.

        def advance(self,requested,dt=.02):
            self.prepare_command(requested)
            return super().advance(requested,dt)

    return ImmediateCenterReference
