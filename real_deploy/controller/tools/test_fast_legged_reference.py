"""Parity with immutable dynamic-period/swing-settling source; no hardware."""
from pathlib import Path
import importlib.util,sys,unittest,time,json
import numpy as np
from fast_command_reference import fast_goal
from instant_center_reference import immediate_center_reference
B=Path(__file__).resolve().parents[2]/'policies/newurdf_legged_a3_1000'
sys.path.insert(0,str(B))
spec=importlib.util.spec_from_file_location('legged_reference_parity',B/'command_reference.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
class FastLeggedTests(unittest.TestCase):
 def test_dynamic_period_reversals_zero_and_all_directions(self):
  p=immediate_center_reference(m.CommandReference)(B/'command_gait_table.npz')
  for target in ([1.5,0,0],[0,-1.5,1.5],[0,0,0],[-1.5,1.5,-1.5],[0,0,0]):
   for k in range(180):
    p.prepare_command(target)
    if k%9==0:
     before=(p.command.copy(),p.gait_command.copy(),p.phase,p.heading,p.position.copy())
     np.testing.assert_allclose(fast_goal(p,target),p.goal(target),rtol=1e-11,atol=1e-11)
     np.testing.assert_array_equal(p.command,before[0]);np.testing.assert_array_equal(p.gait_command,before[1])
     self.assertEqual((p.phase,p.heading),before[2:4]);np.testing.assert_array_equal(p.position,before[4])
    p.advance(target)
 def test_random_states_include_distinct_settling_and_edge_phase(self):
  p=immediate_center_reference(m.CommandReference)(B/'command_gait_table.npz');rng=np.random.default_rng(1110)
  for k in range(200):
   p.command=rng.uniform(-1.5,1.5,3);p.command[:2]*=min(1.,1.5/max(1e-12,np.linalg.norm(p.command[:2])))
   p.gait_command=rng.uniform(-1.5,1.5,3);p.phase=np.nextafter(1.,0.) if k%3==0 else rng.random();p.heading=rng.uniform(-10,10)
   p.immediate_zero=bool(k%2);target=np.zeros(3) if k%4==0 else rng.uniform(-2,2,3);p.prepare_command(target)
   np.testing.assert_allclose(fast_goal(p,target),p.goal(target),rtol=1e-11,atol=1e-11)
if __name__=='__main__':unittest.main()
