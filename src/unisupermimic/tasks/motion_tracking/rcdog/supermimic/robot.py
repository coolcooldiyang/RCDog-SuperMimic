"""Joint-output-side RCDog parameters; physical source URDF is retained."""
RCDOG_JOINT_NAMES = ['rr_hip_joint', 'rr_thigh_joint', 'rr_calf_joint', 'rl_hip_joint', 'rl_thigh_joint', 'rl_calf_joint', 'fr_hip_joint', 'fr_thigh_joint', 'fr_calf_joint', 'fl_hip_joint', 'fl_thigh_joint', 'fl_calf_joint']
TRACKED_BODIES = ['base_link', 'rr_thigh', 'rr_calf', 'rr_foot', 'rl_thigh', 'rl_calf', 'rl_foot', 'fr_thigh', 'fr_calf', 'fr_foot', 'fl_thigh', 'fl_calf', 'fl_foot']
DEFAULT_JOINT_POS = [-0.018335107193686445, -0.13244422550420512, -0.0011704234354192743, 0.02072457668668584, 0.1330973745532801, -4.4055163179920176e-05, 0.01833510666804733, -0.13214757138346628, -0.001166977560049921, -0.020724575436395665, 0.13280004814541338, -4.433912579885794e-05]
KP = [60.0]*12
KD = [2.0]*12
EFFORT = [23.7,23.7,42.66]*4
VELOCITY_LIMIT = [30.1,30.1,15.05]*4
GEAR_RATIO = [1.0,1.0,2.0]*4
COLLISION_GEOM_NAMES = ('base_collision_00', 'base_collision_01', 'base_collision_02', 'base_collision_03', 'base_collision_04', 'base_collision_05', 'base_collision_06', 'base_collision_07', 'base_collision_08', 'base_collision_09', 'base_collision_10', 'base_collision_11', 'base_collision_12', 'rr_hip_collision', 'rr_thigh_collision', 'rr_calf_collision', 'rr_foot_collision', 'rl_hip_collision', 'rl_thigh_collision', 'rl_calf_collision', 'rl_foot_collision', 'fr_hip_collision', 'fr_thigh_collision', 'fr_calf_collision', 'fr_foot_collision', 'fl_hip_collision', 'fl_thigh_collision', 'fl_calf_collision', 'fl_foot_collision')
def action_scale(joint_name):
    if joint_name not in RCDOG_JOINT_NAMES: raise KeyError(joint_name)
    return 0.35
def default_joint_pos_list(): return DEFAULT_JOINT_POS.copy()
