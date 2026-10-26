"""Seeded perturbation trials from frame zero using the full coupled runtime."""
import argparse,json
from pathlib import Path
import torch
from unisupermimic.tasks.motion_tracking.rpo.supermimic.evaluate import make_eval_env,evaluate
from unisupermimic.rl.algos.supermimic.geometry import quat_to_mat,so3_log
from unisupermimic.rl.contracts.supermimic import SUPERMIMIC_TRANSITION_KEY

@torch.inference_mode()
def main():
    p=argparse.ArgumentParser();p.add_argument('--ckpt',type=Path,required=True)
    p.add_argument('--trials',type=int,default=8);p.add_argument('--scale',type=float,default=.15)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--fp32',action='store_true')
    a=p.parse_args();env=make_eval_env(a.ckpt.parent,backend='mujoco')
    checkpoint=torch.load(a.ckpt,map_location=env.device,weights_only=True)
    if a.fp32:checkpoint['args']['amp']='none'
    command=env.command_manager.get_term('motion');results=[]
    states=[];original_step=env.step
    def recorded_step(actions):
        state=original_step(actions);transition=state.info[SUPERMIMIC_TRANSITION_KEY]
        if not states:states.append(transition['s'][0].detach().cpu().clone())
        states.append(transition['s1'][0].detach().cpu().clone())
        return state
    env.step=recorded_step
    try:
        command.reset_noise=True;command.cfg.reset_noise_scale=a.scale
        for seed in range(a.trials):
            env.reset(seed=seed+101)
            states.clear()
            result=evaluate(env,checkpoint);result.update(seed=seed+101)
            actual=torch.stack(states);rotation=quat_to_mat(actual[:,3:7])
            accumulated=so3_log(rotation[:-1].transpose(-1,-2)@rotation[1:]).sum(0)
            delta=actual[-1,:3]-actual[0,:3]
            reference=command.motion.root_p[-1]-command.motion.root_p[0]
            progress=float((delta[:2].to(reference.device)*reference[:2]).sum()/reference[:2].square().sum().clamp(min=1e-8))
            end_speed=float(actual[-25:,7:10].norm(dim=-1).mean())
            is_backflip='backflip' in command.motion.names[0]
            semantic_pass=(float(accumulated[1]) < -5. and end_speed < .25) if is_backflip else progress>.8
            result.update(body_rotation_integral_rad=accumulated.tolist(),forward_progress_fraction=progress,
                          final_half_second_mean_root_speed_m_s=end_speed,
                          motion_completed=bool(result['n_failed']==0 and semantic_pass))
            results.append(result);print(json.dumps(result),flush=True)
    finally:env.close();env.backend.close()
    report={'checkpoint':str(a.ckpt),'reset_noise_scale':a.scale,'fp32':a.fp32,'trials':results,
            'passed':sum(r['motion_completed'] for r in results),'total':len(results)}
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({'passed':report['passed'],'total':report['total']}))

if __name__=='__main__':main()
