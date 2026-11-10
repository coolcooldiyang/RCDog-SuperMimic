"""Actual exported actor/reference, no serial devices or motor output."""
from pathlib import Path
import unittest
from unittest.mock import Mock
import numpy as np
from deploy_newurdf import ContinuousActor
from fast_command_reference import fast_goal

ROOT=Path(__file__).resolve().parents[1]


class InstantCenterTests(unittest.TestCase):
    def setUp(self):
        self.actor=ContinuousActor(ROOT/'../policies/newurdf_command150_final500')
        self.actor.infer=Mock(return_value=(np.zeros(12),self.actor.q0.copy()))

    def tick(self,command,**kwargs):
        return self.actor.tick_command(self.actor.q0,np.zeros(12),np.eye(3),np.zeros(3),command,**kwargs)

    def test_neutral_zeroes_reference_in_same_tick_without_reset(self):
        a=self.actor;p=a.planner
        p.command[:]=[1.2,-.7,1.1];p.phase=.4;p.heading=.7
        position=p.position.copy();a.history.append(np.ones(48));previous=a.previous.copy()
        self.tick([0,0,0])
        self.assertIs(a.planner,p)
        np.testing.assert_array_equal(p.command,np.zeros(3))
        np.testing.assert_array_equal(p.position,position)
        self.assertEqual(p.heading,.7)
        self.assertAlmostEqual(p.phase,(.4+.02/p.period)%1)
        self.assertEqual(len(a.history),1)
        np.testing.assert_array_equal(a.previous,previous)
        expected=fast_goal(p,[0,0,0])
        self.assertTrue(np.isfinite(expected).all())

    def test_new_push_responds_next_tick_without_stop_cooldown(self):
        self.actor.planner.command[:]=[1,0,0]
        self.tick([0,0,0]);self.tick([0,-.5,.5])
        self.assertLess(self.actor.planner.command[1],0)
        self.assertGreater(self.actor.planner.command[2],0)

    def test_explicit_exit_keeps_controlled_deceleration(self):
        p=self.actor.planner;p.command[:]=[1.,0,.5]
        self.tick([0,0,0],immediate_zero=False)
        np.testing.assert_allclose(p.command,[.99,0,.48])
        copy=p.copy();self.assertFalse(copy.immediate_zero)

    def test_lookahead_copies_follow_same_zero_rule(self):
        p=self.actor.planner;p.command[:]=[.8,.5,-.5]
        copy=p.copy();copy.advance([0,0,0])
        np.testing.assert_array_equal(copy.command,np.zeros(3))
        np.testing.assert_allclose(p.command,[.8,.5,-.5])
        p.prepare_command([0,0,0])
        np.testing.assert_allclose(fast_goal(p,[0,0,0]),p.goal([0,0,0]),atol=1e-12)


if __name__=='__main__':unittest.main()
