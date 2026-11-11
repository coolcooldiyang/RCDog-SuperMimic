"""Bounded scheduling recovery, with no serial devices or motor output."""
import unittest
from collections import deque
from unittest.mock import Mock,patch
from deploy_newurdf import NewDeployment
from deploy_050 import Deployment,FeedbackStaleError


class FeedbackWaitTests(unittest.TestCase):
    def setUp(self):
        self.now=0.
        self.d=NewDeployment.__new__(NewDeployment)
        self.d.cfg={'joint_age_max_s':.012,'imu_age_max_s':.02}
        self.d.imu=Mock()
        self.d.imu.snapshot.return_value={'age_s':.001}
        self.d.feedback_wait_events=deque(maxlen=64)
        self.d.require_ready=Mock()

    def sleep(self,dt):self.now+=dt

    def run_wait(self,fn):
        with patch('deploy_newurdf.time.perf_counter',lambda:self.now),patch('deploy_newurdf.time.sleep',self.sleep):
            return fn()

    def test_after_compute_recovery_beyond_old_two_ms_needs_fresh_sample(self):
        self.d.require_ready.side_effect=lambda:[{'feedback_age_s':.013+self.now if self.now<.004 else .001}]
        age,_,wait=self.run_wait(self.d.fresh_feedback_after_compute)
        self.assertEqual(age,.001)
        self.assertGreaterEqual(wait,4.)
        self.assertEqual(len(self.d.feedback_wait_events),1)

    def test_pre_compute_retries_but_never_returns_old_state(self):
        fresh=object()
        self.d.require_ready.return_value=[{'feedback_age_s':.014}]
        def state(**kwargs):
            if self.now<.004:raise FeedbackStaleError('old')
            return fresh
        with patch.object(Deployment,'state',side_effect=state):
            self.assertIs(self.run_wait(self.d.state),fresh)

    def test_wait_cannot_cross_twenty_ms_feedback_deadline(self):
        self.d.require_ready.side_effect=lambda:[{'feedback_age_s':.018+self.now}]
        with self.assertRaisesRegex(FeedbackStaleError,'20毫秒硬时限'):
            self.run_wait(self.d.fresh_feedback_after_compute)
        self.assertLessEqual(self.now,.0025)

    def test_wait_has_eight_ms_budget_even_if_samples_keep_trickling(self):
        self.d.require_ready.return_value=[{'feedback_age_s':.013}]
        with self.assertRaisesRegex(FeedbackStaleError,'12毫秒'):
            self.run_wait(self.d.fresh_feedback_after_compute)
        self.assertLessEqual(self.now,.0081)

    def test_imu_hard_expiry_is_not_retried(self):
        self.d.require_ready.return_value=[{'feedback_age_s':.001}]
        self.d.imu.snapshot.return_value={'age_s':.031}
        with self.assertRaisesRegex(RuntimeError,'IMU'):
            self.run_wait(self.d.fresh_feedback_after_compute)
        self.assertEqual(self.now,0.)

    def test_soft_imu_delay_waits_for_fresh_frame(self):
        self.d.require_ready.return_value=[{'feedback_age_s':.001}]
        self.d.imu.snapshot.side_effect=lambda:{'age_s':.021+self.now if self.now<.003 else .001}
        _,age,wait=self.run_wait(self.d.fresh_feedback_after_compute)
        self.assertEqual(age,.001)
        self.assertGreaterEqual(wait,3.)
        self.assertLessEqual(wait,8.)

    def test_stalled_imu_still_fails_within_wait_budget(self):
        self.d.require_ready.return_value=[{'feedback_age_s':.001}]
        self.d.imu.snapshot.return_value={'age_s':.021}
        with self.assertRaisesRegex(RuntimeError,'20毫秒'):
            self.run_wait(self.d.fresh_feedback_after_compute)
        self.assertLessEqual(self.now,.0081)

    def test_imu_reader_error_is_not_retried(self):
        self.d.require_ready.return_value=[{'feedback_age_s':.001}]
        self.d.imu.snapshot.side_effect=RuntimeError('IMU disconnected')
        with self.assertRaisesRegex(RuntimeError,'disconnected'):
            self.run_wait(self.d.fresh_feedback_after_compute)
        self.assertEqual(self.now,0.)

    def test_before_compute_uses_the_fresh_frame_it_checked(self):
        self.d.require_ready.return_value=[{'feedback_age_s':.001}]
        fresh={'age_s':.001,'tag':'fresh'}
        self.d.imu.snapshot.side_effect=lambda:({'age_s':.021} if self.now<.003 else fresh)
        with patch.object(Deployment,'state',return_value='state') as state:
            self.assertEqual(self.run_wait(self.d.state),'state')
            state.assert_called_once_with(imu_sample=fresh)
        self.assertGreaterEqual(self.now,.003)

    def test_motor_error_is_not_retried(self):
        self.d.require_ready.side_effect=RuntimeError('电机故障')
        with self.assertRaisesRegex(RuntimeError,'电机故障'):
            self.run_wait(self.d.fresh_feedback_after_compute)
        self.assertEqual(self.now,0.)


if __name__=='__main__':unittest.main()
