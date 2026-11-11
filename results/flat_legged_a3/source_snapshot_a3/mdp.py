# RPO adaptation, 2026-10-26; derived from Uni-SuperMimic where applicable.
# RPO extension / combined software: GPL-3.0-only. See LICENSE and THIRD_PARTY.md.
"""Differentiable SuperMimic observations, goals, rewards, and terminations."""

from __future__ import annotations

import copy
import os
from dataclasses import dataclass
from typing import Literal, overload

import torch

from unisupermimic.rl.algos.supermimic.geometry import (
    invariant_features,
    mat_to_6d,
    rot_angle_sq,
    yaw_mat,
)
from unisupermimic.rl.contracts.supermimic import State

from .kinematics import RCDogKinematics
from .motion import CONTROL_DT, MotionLib
from . import terrain
from .robot import VELOCITY_LIMIT
REFERENCE_BEZIER = os.environ.get('RCDOG_REFERENCE_BEZIER','0') == '1'
FLAT_GAIT = os.environ.get('RCDOG_FLAT_GAIT','0') == '1'
FLAT_SLIP_V3 = os.environ.get('RCDOG_FLAT_SLIP_V3','0') == '1'
POSITION_RATE_LIMIT = os.environ.get('RCDOG_POSITION_RATE_LIMIT','0') == '1'

STATE_DIM = 37
NUM_J = 12
OBS_DIM = 12 + 12 + 3 + 3 + 12 + 6
GOAL_FRAME_DIM = 39
XI_DIM = 16
INV_DIM = 1 + 6 + 3 + 3 + 12 + 12
CRITIC_EXTRA = 12 + 13 * 3 + 3 + 6 + XI_DIM

OBS_NOISE = torch.cat(
    (
        torch.full((12,), 0.01),
        torch.full((12,), 0.5),
        torch.full((3,), 0.2),
        torch.full((3,), 0.05),
        torch.zeros(12),
        torch.full((6,), 0.05),
    )
)
OBS_SCALE = torch.cat(
    (
        torch.ones(12),
        torch.full((12,), 0.1),
        torch.full((3,), 0.25),
        torch.ones(3),
        torch.ones(12),
        torch.ones(6),
    )
)
MAX_STEP_REWARD = 10.0 * CONTROL_DT


@dataclass
class RewardCfg:
    body_pos_std: float = 0.12
    body_ori_std: float = 0.4
    body_linvel_std: float = 0.5
    body_angvel_std: float = 3.14
    root_pos_std: float = 0.15
    root_ori_std: float = 0.4
    w_action_rate: float = -0.05
    w_joint_limit: float = -2.0
    soft_limit_factor: float = 0.9
    term_height: float = 0.10
    w_upright: float = -1.0
    w_excess_tilt: float = -3.0
    w_roll_pitch_rate: float = -0.08
    w_stop_joint_speed: float = -0.8
    w_stop_action_rate: float = -1.0
    w_stop_body_speed: float = -2.0
    term_tilt: float = 0.09369221296335006  # 1-cos(25 deg), below hardware 30 deg


class MDP:
    num_joints = NUM_J
    context_dim = XI_DIM
    obs_dim = OBS_DIM
    goal_frame_dim = GOAL_FRAME_DIM
    max_step_reward = MAX_STEP_REWARD

    def __init__(
        self,
        kin: RCDogKinematics,
        motion: MotionLib,
        default_q: torch.Tensor,
        cfg: RewardCfg | None = None,
        coupled_limits: bool = False,
    ):
        if default_q.shape != (NUM_J,):
            raise ValueError(
                f"SuperMimic default_q must have shape {(NUM_J,)}, got {default_q.shape}"
            )
        self.kin = kin
        self.motion = motion
        self.cfg = RewardCfg() if cfg is None else cfg
        self.coupled_limits = coupled_limits
        self.default_q = default_q
        middle = (kin.q_lo + kin.q_hi) / 2
        half_range = (kin.q_hi - kin.q_lo) / 2
        self.soft_lo = middle - half_range * self.cfg.soft_limit_factor
        self.soft_hi = middle + half_range * self.cfg.soft_limit_factor
        device = default_q.device
        self.obs_noise = OBS_NOISE.to(device)
        self.obs_scale = OBS_SCALE.to(device)

    def to(self, device: str | torch.device) -> MDP:
        """Return a copy on ``device``; the original instance is left untouched."""
        device = torch.device(device)
        if device == self.default_q.device:
            return self
        new = copy.copy(self)
        new.kin = self.kin.to(device)
        new.motion = self.motion.to(device)
        for name in ("default_q", "soft_lo", "soft_hi", "obs_noise", "obs_scale"):
            setattr(new, name, getattr(self, name).to(device))
        return new

    def obs(
        self,
        state: State,
        previous_action: torch.Tensor,
        context: torch.Tensor,
        frames: torch.Tensor,
    ) -> torch.Tensor:
        """Return the 81D noise-free proprioceptive observation."""
        _, rotation, _, angular_velocity, joint_pos, joint_vel = state
        rotation_t = rotation.transpose(-1, -2)
        default_offset = context[..., 1:13]
        gravity = -rotation[..., 2, :]
        relative_rotation = mat_to_6d(rotation_t @ self.motion.root_R[frames])
        return torch.cat(
            (
                joint_pos - self.default_q - default_offset,
                joint_vel,
                (rotation_t @ angular_velocity.unsqueeze(-1)).squeeze(-1),
                gravity,
                previous_action,
                relative_rotation,
            ),
            -1,
        )

    def limit_action(self, action: torch.Tensor, previous_action: torch.Tensor, context: torch.Tensor) -> torch.Tensor:
        """Saturate desired position velocity; never modify measured joint state."""
        if not POSITION_RATE_LIMIT:
            return action
        action = action.float()
        previous_action = previous_action.float()
        base = self.default_q + context[..., 1:13]
        desired = (base + .25*action).clamp(-1.55, 1.55)
        desired_action = (desired-base)/.25
        delta = torch.tensor(VELOCITY_LIMIT, device=action.device, dtype=action.dtype)*CONTROL_DT/.25
        return previous_action + torch.maximum(torch.minimum(desired_action-previous_action, delta), -delta)

    def add_noise(self, obs: torch.Tensor) -> torch.Tensor:
        return obs + (2 * torch.rand_like(obs) - 1) * self.obs_noise

    def goal(self, frames: torch.Tensor, num_frames: int) -> torch.Tensor:
        return self.motion.goal(frames, num_frames)

    @overload
    def reward(
        self,
        state: State,
        frames: torch.Tensor,
        action: torch.Tensor,
        previous_action: torch.Tensor,
        terms: Literal[False] = False,
    ) -> torch.Tensor: ...

    @overload
    def reward(
        self,
        state: State,
        frames: torch.Tensor,
        action: torch.Tensor,
        previous_action: torch.Tensor,
        terms: Literal[True],
    ) -> tuple[torch.Tensor, dict[str, torch.Tensor]]: ...

    def reward(
        self,
        state: State,
        frames: torch.Tensor,
        action: torch.Tensor,
        previous_action: torch.Tensor,
        terms: bool = False,
    ) -> torch.Tensor | tuple[torch.Tensor, dict[str, torch.Tensor]]:
        """Compare a state with its reference and return the dt-weighted reward."""
        cfg, motion = self.cfg, self.motion
        position, rotation, linear_velocity, angular_velocity, joint_pos, joint_vel = state
        body_p, body_R, body_v, body_w = self.kin.forward(
            position,
            rotation,
            linear_velocity,
            angular_velocity,
            joint_pos,
            joint_vel,
        )
        ref_p, ref_R = motion.root_p[frames], motion.root_R[frames]
        if terrain.ENABLED:
            ref_p = torch.cat((ref_p[..., :2], (.35+terrain.support_height(position)).unsqueeze(-1)), -1)
        delta_rotation = yaw_mat(rotation @ ref_R.transpose(-1, -2))
        anchor = torch.cat((position[..., :2], ref_p[..., 2:3]), -1)
        ref_body_p = anchor.unsqueeze(-2) + (
            delta_rotation.unsqueeze(-3)
            @ (motion.body_p[frames] - motion.root_p[frames].unsqueeze(-2)).unsqueeze(-1)
        ).squeeze(-1)
        ref_body_R = delta_rotation.unsqueeze(-3) @ motion.body_R[frames]
        if terrain.ENABLED and terrain.REWARD_V2:
            # The old common-height foot reference pulled stance feet toward
            # a plane even when front and rear feet stood on different steps.
            feet_idx = [3, 6, 9, 12]
            lane = position[..., 1:2] > -2
            local_ref_z = terrain.height(body_p[..., feet_idx, :2]) + motion.body_p[frames][..., feet_idx, 2]
            ref_body_p = ref_body_p.clone()
            ref_body_p[..., feet_idx, 2] = torch.where(lane, local_ref_z, ref_body_p[..., feet_idx, 2])
        error_pos = ((ref_body_p - body_p) ** 2).sum(-1).mean(-1)
        error_ori = rot_angle_sq(ref_body_R, body_R).mean(-1)
        error_linvel = ((motion.body_v[frames] - body_v) ** 2).sum(-1).mean(-1)
        error_angvel = ((motion.body_w[frames] - body_w) ** 2).sum(-1).mean(-1)
        error_root_pos = ((ref_p - position) ** 2).sum(-1)
        error_root_ori = rot_angle_sq(ref_R, rotation)
        tracking = (
            torch.exp(-error_pos / cfg.body_pos_std**2)
            + torch.exp(-error_ori / cfg.body_ori_std**2)
            + torch.exp(-error_linvel / cfg.body_linvel_std**2)
            + torch.exp(-error_angvel / cfg.body_angvel_std**2)
            + 0.5 * torch.exp(-error_root_pos / cfg.root_pos_std**2)
            + 0.5 * torch.exp(-error_root_ori / cfg.root_ori_std**2)
        )
        action_rate = ((action - previous_action) ** 2).sum(-1)
        joint_limit = (
            torch.relu(joint_pos - self.soft_hi) + torch.relu(self.soft_lo - joint_pos)
        ).sum(-1)
        reward = (
            tracking + cfg.w_action_rate * action_rate + cfg.w_joint_limit * joint_limit
        ) * CONTROL_DT
        # The four sole origins are tracked indices 3,6,9,12.
        feet = [3, 6, 9, 12]
        foot_p, ref_foot = body_p[..., feet, :], ref_body_p[..., feet, :]
        if terrain.ENABLED:
            # Phase comes from the same flat reference supplied at deployment.
            flat_z = motion.body_p[frames][..., feet, 2]
            swing = motion.swing[frames] if REFERENCE_BEZIER else ((flat_z-.02)/.025).clamp(0,1)
            terrain_z = terrain.height(foot_p[..., :2])
            terrain_lane = (position[..., 1:2] > -2).to(swing.dtype)
            target_z = terrain_z+.02+(.025+.075*terrain_lane)*swing
            if terrain.REWARD_V2:
                target_z = torch.where(terrain_lane.bool(), terrain.swing_clearance_target(foot_p, swing, motion.root_v[frames], motion.swing_lift[frames] if REFERENCE_BEZIER else .04), target_z)
            foot_z = (foot_p[..., 2]-target_z).square().mean(-1)
        else:
            foot_z = ((foot_p[..., 2]-ref_foot[..., 2])**2).mean(-1)
        foot_xy = ((foot_p[..., :2]-ref_foot[..., :2])**2).sum(-1).mean(-1)
        stance = motion.stance[frames] if REFERENCE_BEZIER else torch.exp(-((motion.body_p[frames][..., feet, 2]-.02)/.012)**2)
        slip = (body_v[..., feet, :2].square().sum(-1)*stance).mean(-1)
        speeds = torch.tensor(VELOCITY_LIMIT if REFERENCE_BEZIER else [8.,10.,10.]*4,device=joint_vel.device)
        overspeed = torch.relu(joint_vel.abs()/(.8*speeds)-1).square().mean(-1)
        reward = reward + (2*torch.exp(-foot_z/.025**2)
                            +torch.exp(-foot_xy/.05**2)-1.0*slip-1.0*overspeed)*CONTROL_DT
        if FLAT_GAIT:
            # Reference phase labels stay independent of measured foot height.
            # Increase world-frame stance-foot slip cost and penalize missing
            # the requested 8cm swing without rewarding exaggerated high steps.
            shortfall = (torch.relu(ref_foot[..., 2]-foot_p[..., 2]-.01)/.04).square()
            swing_weight = motion.swing[frames]
            swing_shortfall = (shortfall*swing_weight).mean(-1)
            # Reference timing misses early touchdown and delayed liftoff. Use
            # a smooth measured-height weight as well, without contact sensors
            # in the actor or non-differentiable contact labels in imagination.
            near_ground = torch.sigmoid((.035-foot_p[..., 2])/.005)
            ground_slip = (body_v[..., feet, :2].square().sum(-1)*near_ground).mean(-1)
            if FLAT_SLIP_V3:
                # Penalize only excess near-ground foot speed. The strong v2
                # all-speed cost traded away directional tracking during tuning.
                foot_xy_speed = torch.sqrt(body_v[..., feet, :2].square().sum(-1)+1e-8)
                excess_ground_slip = (torch.relu(foot_xy_speed-.1).square()*near_ground).mean(-1)
                ref_xy = motion.root_v[frames, :2]
                ref_norm = torch.linalg.vector_norm(ref_xy, dim=-1)
                direction = ref_xy/ref_norm.clamp_min(.05).unsqueeze(-1)
                cross_velocity = linear_velocity[...,0]*direction[...,1]-linear_velocity[...,1]*direction[...,0]
                moving = ref_norm.square()/(ref_norm.square()+.05**2)
                reward = reward - (9.*slip+10.*excess_ground_slip+.6*swing_shortfall
                                    +.5*moving*(cross_velocity/.15).square())*CONTROL_DT
            else:
                reward = reward - (19.*slip+20.*ground_slip+.6*swing_shortfall)*CONTROL_DT
        if terrain.ENABLED and terrain.REWARD_V2:
            # Dense clearance gradients survive a missed step; punish abrupt
            # targets before the physical joint reaches the unchanged limit.
            active = terrain_lane.squeeze(-1)
            clearance_error = (foot_p[..., 2]-target_z).square()
            per_foot_bonus = torch.exp(-clearance_error/.025**2).mean(-1)
            clearance_shortfall = (torch.relu(target_z-foot_p[..., 2])/.05).square().mean(-1)
            target_jump = (.25*(action-previous_action)).abs().amax(-1)
            jump_cost = torch.relu(target_jump/.12-1).square()
            early_speed_cost = torch.relu(joint_vel.abs()/(.65*speeds)-1).square().mean(-1)
            reward = reward + active*(2*(per_foot_bonus-torch.exp(-foot_z/.025**2))
                                      -.5*clearance_shortfall-.3*jump_cost-2*early_speed_cost)*CONTROL_DT
        if terrain.ENABLED and terrain.REWARD_V3:
            active = terrain_lane.squeeze(-1)
            calf_cost = terrain.calf_clearance_cost(body_p[..., [2,5,8,11], :], foot_p)
            reward = reward - 2.*active*calf_cost*CONTROL_DT
        # Explicit translation and yaw-rate tracking make standing a poor
        # solution for small lateral and turning references.
        root_velocity_error = (linear_velocity[..., :2]-motion.root_v[frames, :2]).square().sum(-1) if terrain.ENABLED else (linear_velocity-motion.root_v[frames]).square().sum(-1)
        yaw_rate_error = (angular_velocity[..., 2]-motion.root_w[frames, 2]).square()
        reward = reward + (4.0*torch.exp(-root_velocity_error/.35**2)-0.5*root_velocity_error
                           +torch.exp(-yaw_rate_error/.3**2))*CONTROL_DT
        if terrain.ENABLED and terrain.REWARD_V3:
            ref_xy = motion.root_v[frames, :2]
            ref_norm = torch.linalg.vector_norm(ref_xy, dim=-1)
            direction = ref_xy/ref_norm.clamp_min(.05).unsqueeze(-1)
            cross_velocity = linear_velocity[...,0]*direction[...,1]-linear_velocity[...,1]*direction[...,0]
            moving = ref_norm.square()/(ref_norm.square()+.05**2)
            # Works for forward, reverse and lateral commands, in world axes.
            reward = reward - .5*moving*(cross_velocity/.15).square()*CONTROL_DT
        # Absolute uprightness, independent of yaw; small lean is allowed.
        tilt_sin_sq = rotation[..., 2, :2].square().sum(-1)
        excess_tilt = torch.relu((0.9271838546 if terrain.ENABLED else 0.9659258263)-rotation[..., 2, 2]) / 0.0596180393
        body_omega = (rotation.transpose(-1,-2) @ angular_velocity.unsqueeze(-1)).squeeze(-1)
        roll_pitch_rate = body_omega[..., :2].square().sum(-1)
        # Only penalize stillness where the reference is genuinely stationary;
        # slow swing phases must not accidentally be treated as standstill.
        ref_speed_sq = motion.root_v[frames].square().sum(-1)
        ref_yaw_sq = motion.root_w[frames,2].square()
        ref_joint_speed_sq = motion.qd[frames].square().mean(-1)
        stop_weight = torch.exp(-ref_speed_sq/.06**2-ref_yaw_sq/.12**2-ref_joint_speed_sq/.25**2)
        stop_joint_speed = stop_weight*joint_vel.square().mean(-1)
        stop_action_rate = stop_weight*action_rate
        stop_body_speed = stop_weight*(linear_velocity.square().sum(-1)+.1*angular_velocity.square().sum(-1))
        stability = ((.2 if terrain.ENABLED else 1.)*cfg.w_upright*tilt_sin_sq/.2588190451**2
                     +cfg.w_excess_tilt*excess_tilt.square()
                     +cfg.w_roll_pitch_rate*roll_pitch_rate
                     +cfg.w_stop_joint_speed*stop_joint_speed
                     +cfg.w_stop_action_rate*stop_action_rate
                     +cfg.w_stop_body_speed*stop_body_speed)
        reward = reward + stability*CONTROL_DT
        if terms:
            return reward, {
                "track": tracking * CONTROL_DT,
                "e_body_pos": error_pos,
                "e_body_ori": error_ori,
                "e_root_pos": error_root_pos,
                "action_rate": action_rate,
                "joint_limit": joint_limit,
                "stance_slip_m2_s2": slip,
                "near_ground_slip_m2_s2": ground_slip if FLAT_GAIT else torch.zeros_like(slip),
                "foot_z_error_m2": foot_z,
                "tilt_sin_sq": tilt_sin_sq,
                "stop_weight": stop_weight,
                "stop_joint_speed": stop_joint_speed,
                "stability": stability*CONTROL_DT,
            }
        return reward

    def termination(self, state: State, frames: torch.Tensor) -> torch.Tensor:
        """Return hard termination from root height and tilt deviations."""
        position, rotation = state[0], state[1]
        ref_p, ref_R = self.motion.root_p[frames], self.motion.root_R[frames]
        if terrain.ENABLED:
            ref_p = torch.cat((ref_p[..., :2], (.35+terrain.support_height(position)).unsqueeze(-1)), -1)
        error_height = (ref_p[..., 2] - position[..., 2]).abs()
        error_tilt = (ref_R[..., 2, 2] - rotation[..., 2, 2]).abs()
        speed_limits=torch.tensor(VELOCITY_LIMIT if POSITION_RATE_LIMIT else [8.,10.,10.]*4,device=state[5].device)
        overspeed=(state[5].abs()>speed_limits).any(-1)
        done=(error_height > self.cfg.term_height) | (error_tilt > self.cfg.term_tilt) | overspeed
        return done

    def soft_termination(
        self,
        state: State,
        frames: torch.Tensor,
        height_temperature: float = 0.02,
        tilt_temperature: float = 0.01,
    ) -> torch.Tensor:
        """Return the differentiable imagined-rollout termination probability."""
        position, rotation = state[0], state[1]
        ref_p, ref_R = self.motion.root_p[frames], self.motion.root_R[frames]
        if terrain.ENABLED:
            ref_p = torch.cat((ref_p[..., :2], (.35+terrain.support_height(position)).unsqueeze(-1)), -1)
        error_height = (ref_p[..., 2] - position[..., 2]).abs()
        error_tilt = (ref_R[..., 2, 2] - rotation[..., 2, 2]).abs()
        alive = (1 - torch.sigmoid((error_height - self.cfg.term_height) / height_temperature)) * (
            1 - torch.sigmoid((error_tilt - self.cfg.term_tilt) / tilt_temperature)
        )
        speed_limits=torch.tensor(VELOCITY_LIMIT if POSITION_RATE_LIMIT else [8.,10.,10.]*4,device=state[5].device)
        alive=alive*(1-torch.sigmoid((state[5].abs()-speed_limits)/.25)).prod(-1)
        return 1 - alive

    def critic_features(
        self,
        state: State,
        frames: torch.Tensor,
        normalized_context: torch.Tensor,
    ) -> torch.Tensor:
        """Return invariant state, goal errors, and normalized simulation context."""
        motion = self.motion
        position, rotation, linear_velocity, angular_velocity, joint_pos, joint_vel = state
        heading_t = yaw_mat(rotation).transpose(-1, -2)
        ref_p, ref_R = motion.root_p[frames], motion.root_R[frames]
        if terrain.ENABLED:
            ref_p = torch.cat((ref_p[..., :2], (.35+terrain.support_height(position)).unsqueeze(-1)), -1)
        body_p = self.kin.forward(
            position,
            rotation,
            linear_velocity,
            angular_velocity,
            joint_pos,
            joint_vel,
        )[0]
        delta_rotation = yaw_mat(rotation @ ref_R.transpose(-1, -2))
        anchor = torch.cat((position[..., :2], ref_p[..., 2:3]), -1)
        ref_body_p = anchor.unsqueeze(-2) + (
            delta_rotation.unsqueeze(-3)
            @ (motion.body_p[frames] - motion.root_p[frames].unsqueeze(-2)).unsqueeze(-1)
        ).squeeze(-1)
        body_error = (
            (heading_t.unsqueeze(-3) @ (ref_body_p - body_p).unsqueeze(-1)).squeeze(-1).flatten(-2)
        )
        root_error = (heading_t @ (ref_p - position).unsqueeze(-1)).squeeze(-1)
        relative_rotation = mat_to_6d(rotation.transpose(-1, -2) @ ref_R)
        return torch.cat(
            (
                invariant_features(state),
                motion.q[frames] - joint_pos,
                body_error,
                root_error,
                relative_rotation,
                normalized_context,
                terrain.scan(state) if terrain.ENABLED else position[..., :0],
            ),
            -1,
        )

    def normalize_context(self, context: torch.Tensor) -> torch.Tensor:
        return torch.cat(
            (
                (context[..., :1] - 0.75) / 0.45,
                context[..., 1:13] / 0.01,
                context[..., 13:16] / 0.05,
            ),
            -1,
        )

    def valid_imagination_start(self, frames: torch.Tensor, horizon: int) -> torch.Tensor:
        clip_end = self.motion.clip_end[self.motion.clip_id[frames]]
        return frames + horizon <= clip_end

    def ref_state(self, frames: torch.Tensor) -> State:
        return self.motion.ref_state(frames)

    def clip_info(self, frames: torch.Tensor) -> dict[str, torch.Tensor]:
        """Return owning clip, its inclusive bounds, and the frame's one-second bin."""
        clips = self.motion.clip_id[frames]
        return {
            "clip_id": clips,
            "clip_start": self.motion.clip_start[clips],
            "clip_end": self.motion.clip_end[clips],
            "frame_bin": self.motion.frame_bin[frames],
        }
