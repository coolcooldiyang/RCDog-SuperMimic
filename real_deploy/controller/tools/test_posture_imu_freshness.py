"""No hardware: freshness recovery must not accept stale orientation."""
import unittest
from unittest.mock import Mock,patch
from dog_motion.deploy_inputs import fresh_imu_snapshot
from deploy_050 import Deployment


class PostureImuTests(unittest.TestCase):
    def setUp(self):
        self.now=0.
        self.reader=Mock()

    def run_reader(self):
        def sleep(dt):self.now+=dt
        with patch('dog_motion.deploy_inputs.time.monotonic',lambda:self.now),patch('dog_motion.deploy_inputs.time.sleep',sleep):
            return fresh_imu_snapshot(self.reader,.02)

    def test_fresh_returns_without_wait(self):
        self.reader.snapshot.return_value={'age_s':.01}
        self.assertEqual(self.run_reader()['age_s'],.01)
        self.assertEqual(self.now,0.)

    def test_delayed_publication_requires_fresh_frame(self):
        old={'age_s':.025};new={'age_s':.001}
        self.reader.snapshot.side_effect=[old,old,new]
        self.assertIs(self.run_reader(),new)
        self.assertGreater(self.now,0)

    def test_persistent_miss_has_eight_ms_budget(self):
        self.reader.snapshot.return_value={'age_s':.021}
        with self.assertRaisesRegex(RuntimeError,'未恢复'):self.run_reader()
        self.assertLessEqual(self.now,.0081)

    def test_never_waits_beyond_existing_thirty_ms_expiry(self):
        self.reader.snapshot.side_effect=lambda:{'age_s':.028+self.now}
        with self.assertRaisesRegex(RuntimeError,'未恢复'):self.run_reader()
        self.assertLessEqual(self.now,.0021)

    def test_reader_fault_is_not_retried(self):
        self.reader.snapshot.side_effect=RuntimeError('IMU 时间戳回退')
        with self.assertRaisesRegex(RuntimeError,'时间戳回退'):self.run_reader()
        self.assertEqual(self.reader.snapshot.call_count,1)

    def test_tilt_protection_still_checks_fresh_frame(self):
        d=Deployment.__new__(Deployment)
        d.imu=self.reader;d.recent_samples=[]
        d.cfg={'imu_age_max_s':.02,'imu':{'R_CI':[[1,0,0],[0,1,0],[0,0,1]]},'posture_tilt_limit_deg':30}
        self.reader.snapshot.return_value={'age_s':.001,'quat_wxyz':[0,1,0,0]}
        with self.assertRaisesRegex(RuntimeError,'倾角'):d.check_posture_imu()


if __name__=='__main__':unittest.main()
