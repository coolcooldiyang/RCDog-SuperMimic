"""Offline checks for continuous forward playback and deliberate stick control."""
import unittest
import numpy as np
from dog_motion.deploy_inputs import ForwardStickGate
from dog_motion.reference_playback import validate_forward_loop,forward_loop_frame
from dog_motion.deploy050 import Actor,Transmission,NativePD,GEAR
from motion_cli import ROOT

class StickTests(unittest.TestCase):
    def setUp(self):
        self.g=ForwardStickGate();self.center=[0.]*16;self.forward=self.center.copy()
        self.forward[1]=1.;self.buttons=[0]*32

    def step(self,axes,ready=True,active=False,event=None):
        return self.g.step(axes,self.buttons,ready,active,event)

    def test_requires_neutral_after_stand_or_check(self):
        self.assertIsNone(self.step(self.forward))
        self.step(self.center,ready=False)
        self.assertIsNone(self.step(self.forward))
        self.step(self.center)
        self.assertEqual(self.step(self.forward),'play')

    def test_release_stops_and_holding_never_restarts(self):
        self.assertEqual(self.step(self.center,active=True),'policy-stop')
        self.assertIsNone(self.step(self.forward))
        self.step(self.center)
        self.assertEqual(self.step(self.forward),'play')
        self.assertIsNone(self.step(self.forward,active=True))

    def test_stop_button_requires_release_before_restart(self):
        self.step(self.center)
        self.assertIsNone(self.step(self.forward,active=True,event='policy-stop'))
        self.assertIsNone(self.step(self.forward))
        self.step(self.center)
        self.buttons[4]=1
        self.assertIsNone(self.step(self.forward))

    def test_unsupported_direction_stops(self):
        left=self.center.copy();left[0]=1.
        self.step(self.center)
        self.assertIsNone(self.step(left))
        self.assertEqual(self.step(left,active=True),'policy-stop')

    def test_loop_seam_and_no_reset_to_entry(self):
        # Current reference: 0.32 s = 16 policy frames, after acceleration.
        bundle=ROOT/'../policies/newurdf_command150_final500'
        with np.load(bundle/'references/lat150_forward.npz') as z:
            r={k:z[k].copy() for k in z.files}
        start,end=350,366
        validate_forward_loop(r,start,end)
        self.assertEqual(forward_loop_frame(365,start,end),365)
        self.assertEqual(forward_loop_frame(366,start,end),350)
        self.assertEqual(forward_loop_frame(383,start,end),351)
        for k in ['q_ref','qd_ref','goal','R_ref']:
            np.testing.assert_allclose(r[k][forward_loop_frame(end,start,end)],r[k][end],atol=1e-4)
        r['goal'][end,0]+=1
        with self.assertRaises(ValueError):validate_forward_loop(r,start,end)

    def test_kd45_conversion(self):
        t=Transmission(np.diag([-1.,-1.,-.5]),[0]*3,[0]*3)
        p=NativePD(t,[0]*3,[20]*3,[4.5]*3,[23.7,23.7,42.66])
        c=p.motor_command([0]*3,[0]*3,[0]*3)
        np.testing.assert_allclose(c['kd'],[4.5/GEAR**2]*2+[4.5/(2*GEAR)**2])
        np.testing.assert_array_equal(np.floor(c['kd']*1280),[143,143,35])

if __name__=='__main__':unittest.main()
