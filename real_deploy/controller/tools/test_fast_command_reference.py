"""Numerical equivalence to the immutable exported reference; no hardware access."""
import importlib.util
from pathlib import Path
import unittest
import numpy as np
from fast_command_reference import fast_goal

ROOT=Path(__file__).resolve().parents[1]
BUNDLE=ROOT/'../policies/newurdf_command150_final500'
spec=importlib.util.spec_from_file_location('exported_reference',BUNDLE/'command_reference.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)

class FastReferenceTests(unittest.TestCase):
    def test_new_bundle_full_range_and_reversals(self):
        bundle=ROOT/'../policies/newurdf_command150_final500'
        spec=importlib.util.spec_from_file_location('new_reference',bundle/'command_reference.py')
        mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
        p=mod.CommandReference(bundle/'command_gait_table.npz');rng=np.random.default_rng(150)
        for command in ([1.5,0,0],[-1.5,0,0],[1.5,1.5,1.5],[-1.5,-1.5,-1.5],[0,0,0]):
            for step in range(320):
                if step%20==0:
                    np.testing.assert_allclose(fast_goal(p,command),p.goal(command),rtol=1e-12,atol=1e-12)
                p.advance(command)
        for _ in range(100):
            p.command=rng.uniform(-1.5,1.5,3);p.phase=rng.random();target=rng.uniform(-2,2,3)
            np.testing.assert_allclose(fast_goal(p,target),p.goal(target),rtol=1e-12,atol=1e-12)
        for phase in (0.,.5,np.nextafter(1.,0.)):
            p.phase=phase
            for x in p.grids[0]:
                for y in p.grids[1]:
                    for w in p.grids[2]:
                        p.command=np.array([x,y,w])
                        np.testing.assert_allclose(fast_goal(p,[0,0,0]),p.goal([0,0,0]),rtol=1e-12,atol=1e-12)
    def test_random_commands_and_continuous_phase(self):
        p=module.CommandReference(BUNDLE/'command_gait_table.npz');rng=np.random.default_rng(24)
        for step in range(1000):
            if step%25==0:command=rng.uniform(-.6,.6,3)
            np.testing.assert_allclose(fast_goal(p,command),p.goal(command),rtol=1e-12,atol=1e-12)
            p.advance(command)
    def test_edge_phases_and_commands(self):
        p=module.CommandReference(BUNDLE/'command_gait_table.npz')
        for phase in (0.,.5,np.nextafter(1.,0.)):
            p.phase=phase
            for value in (-.5,0.,.5):
                p.command[:]=value
                for target in ([-.5,-.5,-.5],[0,0,0],[.5,.5,.5]):
                    np.testing.assert_allclose(fast_goal(p,target),p.goal(target),rtol=1e-12,atol=1e-12)

if __name__=='__main__':unittest.main()
