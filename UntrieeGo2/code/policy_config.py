# 观测归一化配置 (从IsaacLab配置中提取)
# 这些是训练时使用的噪声范围和归一化参数

import numpy as np

OBSERVATION_NOISE = {
    'base_lin_vel': (-0.1, 0.1),      # 基座线速度噪声
    'base_ang_vel': (-0.2, 0.2),      # 基座角速度噪声
    'projected_gravity': (-0.05, 0.05), # 投影重力噪声
    'joint_pos': (-0.01, 0.01),        # 关节位置噪声
    'joint_vel': (-1.5, 1.5),          # 关节速度噪声
    'height_scan': (-0.1, 0.1),        # 高度扫描噪声
    'height_scan_clip': (-1.0, 1.0),   # 高度扫描裁剪范围
}

# 关节顺序 (Unitree Go2)
# 根据IsaacLab配置
JOINT_ORDER = [
    # 左前腿
    'LF_hip_joint',   # 髋关节
    'LF_thigh_joint', # 大腿关节
    'LF_calf_joint',  # 小腿关节
    # 右前腿
    'RF_hip_joint',
    'RF_thigh_joint',
    'RF_calf_joint',
    # 左后腿
    'LH_hip_joint',
    'LH_thigh_joint',
    'LH_calf_joint',
    # 右后腿
    'RH_hip_joint',
    'RH_thigh_joint',
    'RH_calf_joint',
]

# 关节默认位置 (用于计算相对位置)
DEFAULT_JOINT_POS = np.array([
    0.0, 0.0, 0.0,  # LF
    0.0, 0.0, 0.0,  # RF
    0.0, 0.0, 0.0,  # LH
    0.0, 0.0, 0.0,  # RH
])

# 动作缩放因子
ACTION_SCALE = 0.25

# 观测维度
OBS_DIM = 235

# 动作维度
ACTION_DIM = 12
