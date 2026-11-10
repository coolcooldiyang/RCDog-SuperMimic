"""Host-side feedback statistics. No hardware access or command generation."""
from collections import deque
import math

DISPLAY_LEGS=('FL','FR','RL','RR')
LEG_NAMES={'FL':'左前 UART1','FR':'右前 UART2','RL':'左后 UART3','RR':'右后 UART4'}
JOINT_NAMES={1:'髋',2:'大腿',3:'小腿'}

class FeedbackStats:
    def __init__(self,window_s=10.):
        self.window_s=window_s
        self.session=None
        self.axes={}
        self.resets=0

    def reset(self):
        self.axes.clear()
        self.resets+=1

    def update(self,status,now):
        session=status.get('session')
        if self.session!=session:
            self.session=session
            self.reset()
        legs=status.get('leg_order',['FL','FR','RR','RL'])
        rows=[]
        for index,m in enumerate(status.get('motors',[])):
            leg=legs[index//3];key=(leg,m['id'])
            valid=int(m.get('valid_frames',0));failed=int(m.get('failed_frames',0))
            age=m.get('feedback_age_s');age_ms=float(age)*1000 if age is not None else None
            history=self.axes.setdefault(key,deque())
            if history and (valid<history[-1][1] or failed<history[-1][2]):
                history.clear();self.resets+=1
            history.append((now,valid,failed,age_ms))
            # Keep only samples inside the requested window; startup uses actual duration.
            while len(history)>1 and history[0][0]<now-self.window_s:history.popleft()
            base=history[0];dv=valid-base[1];df=failed-base[2];dt=now-base[0]
            ages=[h[3] for h in history if h[3] is not None and math.isfinite(h[3])]
            ages.sort()
            rows.append(dict(leg=leg,id=m['id'],joint=JOINT_NAMES.get(m['id'],'?'),online=bool(m.get('online')),err=m.get('err'),temperature=m.get('temp'),age_ms=age_ms,
                poll_gap_ms=m.get('last_poll_gap_ms'),io_ms=m.get('last_io_ms'),
                window_s=dt,valid_delta=dv,failed_delta=df,
                failure_percent=(100.*df/(dv+df)) if dv+df else None,
                success_hz=dv/dt if dt>0 else None,failed_total=failed,
                consecutive_failures=m.get('consecutive_failures',0),
                age_peak_ms=max(ages) if ages else None,
                age_p95_ms=ages[min(len(ages)-1,int(.95*len(ages)))] if ages else None,
                failure_reason=m.get('last_failure_reason'),io_exception=m.get('last_io_exception')))
        rows.sort(key=lambda r:(DISPLAY_LEGS.index(r['leg']),r['id']))
        return rows
