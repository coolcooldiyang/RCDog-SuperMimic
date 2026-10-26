# RPO adaptation, 2026-10-26; derived from Uni-SuperMimic where applicable.
# RPO extension / combined software: GPL-3.0-only. See LICENSE and THIRD_PARTY.md.
"""RPO parameters from the existing deployment snapshot; not G1 motor limits."""
RPO_JOINT_NAMES = ['left_thigh_yaw_joint', 'left_thigh_roll_joint', 'left_thigh_pitch_joint', 'left_knee_joint', 'left_ankle_pitch_joint', 'left_ankle_roll_joint', 'right_thigh_yaw_joint', 'right_thigh_roll_joint', 'right_thigh_pitch_joint', 'right_knee_joint', 'right_ankle_pitch_joint', 'right_ankle_roll_joint', 'torso_joint', 'left_arm_pitch_joint', 'left_arm_roll_joint', 'left_arm_yaw_joint', 'left_elbow_pitch_joint', 'left_elbow_yaw_joint', 'right_arm_pitch_joint', 'right_arm_roll_joint', 'right_arm_yaw_joint', 'right_elbow_pitch_joint', 'right_elbow_yaw_joint']
TRACKED_BODIES = ['base_link', 'left_thigh_roll_link', 'left_knee_link', 'left_ankle_roll_link', 'right_thigh_roll_link', 'right_knee_link', 'right_ankle_roll_link', 'torso_link', 'left_arm_roll_link', 'left_elbow_pitch_link', 'left_elbow_yaw_link', 'right_arm_roll_link', 'right_elbow_pitch_link', 'right_elbow_yaw_link']
DEFAULT_JOINT_POS = [0, 0, -0.1, 0.3, -0.2, 0, 0, 0, -0.1, 0.3, -0.2, 0, 0, 0.18, 0.06, 0, 0.78, 0, 0.18, -0.06, 0, 0.78, 0]
KP = [100, 100, 100, 150, 40, 40, 100, 100, 100, 150, 40, 40, 150, 40, 40, 40, 30, 20, 40, 40, 40, 30, 20]
KD = [3.3, 3.3, 3.3, 5, 2, 2, 3.3, 3.3, 3.3, 5, 2, 2, 5, 2, 2, 2, 1.5, 1, 2, 2, 2, 1.5, 1]

def action_scale(joint_name: str) -> float:
    if joint_name not in RPO_JOINT_NAMES:
        raise KeyError(joint_name)
    return 0.25

def default_joint_pos_list() -> list[float]:
    return DEFAULT_JOINT_POS.copy()

COLLISION_GEOM_NAMES = ('base_collision', 'left_thigh_pitch_collision', 'left_knee_collision', 'left_ankle_roll_collision', 'right_thigh_pitch_collision', 'right_knee_collision', 'right_ankle_roll_collision', 'torso_collision', 'left_arm_pitch_collision', 'left_arm_roll_collision', 'left_arm_yaw_collision', 'left_elbow_pitch_collision', 'right_arm_pitch_collision', 'right_arm_roll_collision', 'right_arm_yaw_collision', 'right_elbow_pitch_collision')
