import unittest
from feedback_stats import FeedbackStats

def state(v=100,f=10,session='a',age=.003):
 return dict(session=session,leg_order=['FL','FR','RR','RL'],motors=[dict(id=i%3+1,online=True,err=0,temp=30,valid_frames=v,failed_frames=f,feedback_age_s=age) for i in range(12)])
class StatsTests(unittest.TestCase):
 def test_increment_rate_ignores_historical_failures(self):
  t=FeedbackStats();self.assertIsNone(t.update(state(),0)[0]['failure_percent'])
  r=t.update(state(298,12),1)[0]
  self.assertEqual(r['failure_percent'],1);self.assertEqual(r['failed_delta'],2);self.assertEqual(r['success_hz'],198)
 def test_no_new_failures_not_historical_total(self):
  t=FeedbackStats();t.update(state(),0);r=t.update(state(300,10),1)[0]
  self.assertEqual(r['failure_percent'],0);self.assertEqual(r['failed_total'],10)
 def test_restart_and_counter_reset_rebaseline(self):
  t=FeedbackStats();t.update(state(),0)
  self.assertIsNone(t.update(state(1,0),1)[0]['failure_percent'])
  self.assertIsNone(t.update(state(500,20,'b'),2)[0]['failure_percent'])
 def test_window_drops_old_failures(self):
  t=FeedbackStats(window_s=10);t.update(state(0,0),0);t.update(state(100,100),1);t.update(state(200,100),9)
  r=t.update(state(400,100),12)[0];self.assertEqual(r['failure_percent'],0);self.assertEqual(r['window_s'],3)
 def test_uart_order(self):
  rows=FeedbackStats().update(state(),0)
  self.assertEqual([r['leg'] for r in rows[::3]],['FL','FR','RL','RR'])
 def test_local_reset_and_no_attempts(self):
  t=FeedbackStats();t.update(state(),0);r=t.update(state(),1)[0];self.assertIsNone(r['failure_percent']);self.assertEqual(r['success_hz'],0)
  t.reset();self.assertIsNone(t.update(state(),2)[0]['failure_percent'])
if __name__=='__main__':unittest.main()
