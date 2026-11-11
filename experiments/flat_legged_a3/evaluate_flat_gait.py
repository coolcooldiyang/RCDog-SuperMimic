"""Measure actual CPU MuJoCo swing clearance and contact-foot slip from rollouts."""
from pathlib import Path
import argparse,json
import numpy as np
import mujoco
from command_reference import CommandReference
R=Path(__file__).resolve().parent


def reference_clearance(model,target):
    planner=CommandReference(R/'reference_candidates/legged_control/command_gait_table.npz')
    planner.command[:]=target;planner.gait_command[:]=target
    planner.period=planner.blend_scalar(planner.period_table)
    planner.duty=planner.blend_scalar(planner.duty_table)
    data=mujoco.MjData(model);ids=[model.body(l+'_foot').id for l in ['rr','rl','fr','fl']]
    z=[]
    for phase in np.linspace(0,1,120,endpoint=False):
        planner.phase=phase;data.qpos[:]=planner.q0;data.qpos[7:]=planner.joints()
        mujoco.mj_forward(model,data);z.append(data.xpos[ids,2].copy())
    return float(np.ptp(z,axis=0).min())


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--stability-report',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    report=json.loads(args.stability_report.read_text());folder=args.stability_report.with_suffix('')
    model=mujoco.MjModel.from_xml_path(str(R.parents[1]/'src/unisupermimic/assets/robots/rcdog/scene_supermimic_motor.xml'))
    ids=[model.body(l+'_foot').id for l in ['rr','rl','fr','fl']];results=[]
    data=mujoco.MjData(model);jp=np.zeros((3,model.nv));jr=np.zeros_like(jp)
    for record in report['results']:
        path=folder/f'{record["case"]}_d{record["delay_ms"]}.npz'
        z=np.load(path);steady=(z['time']>7)&(z['time']<10.5);contacts=[];heights=[];slips=[]
        for q,v in zip(z['qpos'][steady],z['qvel'][steady]):
            data.qpos[:]=q;data.qvel[:]=v;mujoco.mj_forward(model,data)
            contact=np.zeros(4,dtype=bool)
            for c in data.contact:
                bodies=model.geom_bodyid[[c.geom1,c.geom2]]
                for j,bid in enumerate(ids):
                    if bid in bodies:contact[j]=True
            contacts.append(contact);heights.append(data.xpos[ids,2].copy()-.02)
            for j,bid in enumerate(ids):
                if contact[j]:
                    mujoco.mj_jacBody(model,data,jp,jr,bid);slips.append(float(np.linalg.norm((jp@v)[:2])))
        contacts=np.array(contacts);heights=np.array(heights);peaks=[]
        if len(contacts):
            for j in range(4):
                off=~contacts[:,j];start=None
                for k,value in enumerate(off):
                    if value and start is None:start=k
                    if not value and start is not None:
                        if start>0 and k-start>=2:peaks.append(float(heights[start:k,j].max()))
                        start=None
        moving=bool(np.linalg.norm(record['target'])>.05)
        ref=reference_clearance(model,record['target']) if moving else 0.
        median=float(np.median(peaks)) if peaks else 0.
        mean_slip=float(np.mean(slips)) if slips else 999.
        p95=float(np.percentile(slips,95)) if slips else 999.
        agreement=float(np.mean((contacts[:,0]==contacts[:,3])&(contacts[:,1]==contacts[:,2]))) if len(contacts) else 0.
        passed=bool(record['passed'] and mean_slip<.12 and p95<.3 and
            (not moving or (len(peaks)>=4 and median>=min(.05,.6*ref) and agreement>=.55)))
        results.append(dict(case=record['case'],delay_ms=record['delay_ms'],passed=passed,
            reference_clearance_m=ref,median_actual_swing_peak_m=median,complete_swings=len(peaks),
            mean_contact_foot_xy_speed_mps=mean_slip,p95_contact_foot_xy_speed_mps=p95,
            diagonal_contact_agreement=agreement))
    result=dict(checkpoint=report['checkpoint'],all_passed=all(r['passed'] for r in results),
        passed=sum(r['passed'] for r in results),total=len(results),hardware_accessed=False,results=results)
    args.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))


if __name__=='__main__':main()
