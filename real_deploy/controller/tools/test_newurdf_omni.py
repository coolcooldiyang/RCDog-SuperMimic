"""Offline joystick direction and entry checks; no devices are opened."""
import unittest
import threading
from unittest.mock import Mock,patch
import numpy as np
from dog_motion.deploy_inputs import OmniStickGate,omni_command,direction
from deploy_newurdf import NewDeployment
from deploy_050 import Deployment

class OmniTests(unittest.TestCase):
 def test_signed_axes_and_diagonal_cap(self):
  for idx,component in ((1,0),(0,1),(3,2)):
   for sign in (-1,1):
    a=[0.]*4;a[idx]=sign;want=[0.]*3;want[component]=sign*.5
    np.testing.assert_allclose(omni_command(a),want)
  c=omni_command([1,1,0,1]);self.assertAlmostEqual(np.linalg.norm(c[:2]),.5);self.assertEqual(c[2],.5)
 def test_neutral_before_all_directions_and_no_stop_on_direction_change(self):
  for a in ([0,1,0,0],[0,-1,0,0],[1,0,0,0],[-1,0,0,0],[0,0,0,1],[0,0,0,-1],[1,1,0,1]):
   g=OmniStickGate();self.assertIsNone(g.step(a,[],True,False))
   g.step([0]*4,[],True,False);self.assertEqual(g.step(a,[],True,False),'play')
   self.assertIsNone(g.step(a,[],True,True))
   self.assertEqual(g.step([0]*4,[],True,True),'policy-stop')
   self.assertIsNone(g.step(a,[],True,False))
 def test_button_and_not_ready_block(self):
  g=OmniStickGate();g.step([0]*4,[],True,False)
  self.assertIsNone(g.step([1]*4,[1],True,False));self.assertFalse(g.neutral_seen)
  g.step([0]*4,[],False,False);self.assertFalse(g.neutral_seen)
  self.assertIsNone(g.step([1]*4,[],True,True,'emergency-stop'))
 def test_requested_command_and_release(self):
  d=NewDeployment.__new__(NewDeployment);d.decelerating=False;d.mode='PLAY';d.pad=Mock();d.begin_stop=Mock();d.cfg={'commissioning_max_forward_mps':.5,'commissioning_max_yaw_rad_s':.5}
  d.pad.snapshot.return_value=([1,-1,0,-1],[],True)
  np.testing.assert_allclose(d.requested_command(),[-.5/2**.5,.5/2**.5,-.5]);d.begin_stop.assert_not_called()
  d.pad.snapshot.return_value=([0]*4,[],True);np.testing.assert_array_equal(d.requested_command(),[0]*3);d.begin_stop.assert_not_called()
  d.pad.snapshot.return_value=([0,.2,0,0],[],True);self.assertGreater(d.requested_command()[0],0);d.begin_stop.assert_not_called()
  d.decelerating=True;d.pad.snapshot.side_effect=AssertionError('stopping must not restart')
  np.testing.assert_array_equal(d.requested_command(),[0]*3)
 def test_continuous_gate_keeps_zero_active_and_accepts_small_stick(self):
  g=OmniStickGate(stop_on_neutral=False,start_threshold=0.)
  self.assertIsNone(g.step([0,.2,0,0],[],True,False))
  g.step([0]*4,[],True,False)
  self.assertEqual(g.step([0,.2,0,0],[],True,False),'play')
  for axes in ([0]*4,[0,-.2,0,0],[0]*4,[.3,0,0,.4]):
   self.assertIsNone(g.step(axes,[],False,True))
  self.assertIsNone(g.step([0]*4,[],False,True,'lie'))
 def test_neutral_deadzone_is_zero_and_disconnect_not_ignored(self):
  d=NewDeployment.__new__(NewDeployment);d.decelerating=False;d.mode='PLAY';d.pad=Mock();d.cfg={'commissioning_max_forward_mps':.5,'commissioning_max_yaw_rad_s':.5}
  d.pad.snapshot.return_value=([.1,-.1,0,.1],[],True)
  np.testing.assert_array_equal(d.requested_command(),[0]*3)
  d.pad.snapshot.side_effect=RuntimeError('gamepad disconnected')
  with self.assertRaisesRegex(RuntimeError,'disconnected'):d.requested_command()
 def test_explicit_stop_and_lie_still_start_exit(self):
  for command in ('policy-stop','lie'):
   d=NewDeployment.__new__(NewDeployment);d.lock=threading.RLock();d.mode='PLAY';d.decelerating=False;d.pending_lie=False
   d.execute(command)
   self.assertTrue(d.decelerating)
   self.assertIsNotNone(d.stop_requested_at)
   self.assertEqual(d.pending_lie,command=='lie')
 def test_all_direction_labels_reach_parent(self):
  d=NewDeployment.__new__(NewDeployment);d.walk_unlocked=True;d.actor=Mock();d.pad=Mock()
  for a in ([0,1,0,0],[0,-1,0,0],[1,0,0,0],[-1,0,0,0],[0,0,0,1],[0,0,0,-1],[1,1,0,0],[-1,1,0,0],[1,-1,0,0],[-1,-1,0,0]):
   d.pad.snapshot.return_value=(a,[],True)
   with patch.object(Deployment,'begin',return_value={}) as begin:
    d.begin('PLAY',direction(a));begin.assert_called_once_with('PLAY',direction(a))
if __name__=='__main__':unittest.main()
