# IsaacLab到MuJoCo策略部署指南

## 项目概述

本项目尝试将IsaacLab中训练的Unitree Go2四足机器人行走策略部署到MuJoCo仿真环境中。策略训练配置：
- **任务**: `Template-Velocity-Test-Unitree-Go2-v0`
- **策略路径**: `/home/robot/work/LekiwiTest/UntrieeGo2/policy/Rough_Walk_policy.pt`
- **训练迭代**: 1499
- **网络结构**: [512, 256, 128] 三层隐藏层

## 核心文件

### 1. `policy_inference.py` - 策略推理核心
加载IsaacLab训练的PyTorch模型并提供推理接口：

```python
from policy_inference import IsaacLabPolicy, create_observation

# 加载策略
policy = IsaacLabPolicy(policy_path, device='cpu')

# 构建观测 (235维)
obs = create_observation(
    base_lin_vel=np.array([0.1, 0.0, 0.0]),
    base_ang_vel=np.array([0.0, 0.0, 0.0]),
    projected_gravity=np.array([0.0, 0.0, -1.0]),
    velocity_command=np.array([0.5, 0.0, 0.0]),
    joint_pos=np.zeros(12),
    joint_vel=np.zeros(12),
    last_action=np.zeros(12),
    height_scan=np.zeros(187),
)

# 获取动作 (12维关节位置偏移)
action = policy.get_action(obs)
```

### 2. `policy_config.py` - 配置参数
观测归一化参数和关节顺序定义。

### 3. `demo.py` - 简单演示
无需MuJoCo的策略推理演示。

## 技术架构

### IsaacLab训练配置

从 `walk_test_env_cfg.py` 中提取的关键配置：

```python
# 动作配置
joint_pos = mdp.JointPositionActionCfg(
    asset_name="robot",
    joint_names=[".*"],
    scale=0.25,              # 动作缩放
    use_default_offset=True   # 相对于默认位置的偏移
)

# 默认关节位置 (UNITREE_GO2_CFG)
FL: [0.1, 0.8, -1.5]   # 左前腿
FR: [-0.1, 0.8, -1.5]  # 右前腿
RL: [0.1, 1.0, -1.5]   # 左后腿
RR: [-0.1, 1.0, -1.5]  # 右后腿

# PD控制器增益 (DCMotorCfg)
stiffness = 25.0
damping = 0.5
effort_limit = 33.5

# 仿真参数
decimation = 4          # 控制频率 = sim_freq / decimation
dt = 0.005             # 物理时间步
episode_length_s = 20.0
```

### 观测空间 (235维)

```python
观测组成:
- base_lin_vel (3): 基座线速度
- base_ang_vel (3): 基座角速度
- projected_gravity (3): 投影重力向量
- velocity_commands (3): 速度命令
- joint_pos_rel (12): 关节位置(相对于默认)
- joint_vel_rel (12): 关节速度
- last_action (12): 上一个动作
- height_scan (187): 高度扫描
```

### MuJoCo模型差异

**关键发现**: MuJoCo模型与IsaacLab模型存在**运动学差异**：

| 特性 | IsaacLab | MuJoCo (go2.xml) |
|------|----------|------------------|
| 关节顺序 | FL→FR→RL→RR | FR→FL→RR→RL |
| 默认位置 | [0.1, 0.8, -1.5] 等 | [0, 0.9, -1.8] |
| 关节限位 | 与Unitree Go2规格匹配 | 可能不同 |
| PD增益 | 25.0, 0.5 | 需要重新调参 |
| 物理参数 | PhysX引擎 | MuJoCo引擎 |

## 部署挑战与解决方案

### 挑战1: 关节顺序映射

**问题**: MuJoCo和IsaacLab的关节顺序不同。

**解决方案**: 创建双向映射数组
```python
# IsaacLab: FL, FR, RL, RR
# MuJoCo: FR, FL, RR, RL
isaaclab_to_mujoco = [3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8]
mujoco_to_isaaclab = [3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8]
```

### 挑战2: 投影重力计算

**问题**: 策略需要正确的机器人姿态信息。

**解决方案**: 使用四元数旋转计算投影重力
```python
from scipy.spatial.transform import Rotation as R

quat = data.qpos[3:7]  # MuJoCo quaternion (w,x,y,z)
quat_xyzw = [quat[1], quat[2], quat[3], quat[0]]
rotation = R.from_quat(quat_xyzw)
projected_gravity = rotation.inv().apply([0, 0, -1])
```

### 挑战3: 物理引擎差异 (根本问题)

**问题**: 即使使用正确的关节位置和PD增益，机器人在MuJoCo中仍会倒塌。

**测试结果**:
- 使用MuJoCo默认位置[0, 0.9, -1.8]: 高度从0.4m降至0.08m
- 使用IsaacLab位置[0.1, 0.8, -1.5]: 高度从0.42m降至0.31m

**根本原因**:
- 不同物理引擎的动力学特性差异
- 质量分布、惯性参数、摩擦系数不同
- 关节约束和接触模型不同

## 推荐方案

### 方案A: 在IsaacLab中部署 (推荐)

**原因**: 策略是在IsaacLab中训练的，在相同环境中部署最可靠。

```bash
cd /home/robot/work/BiShe/IsaacLabBisHe/source/MyProject

# 使用IsaacLab的play脚本
python ../../../../scripts/play.py \
    --task=Template-Velocity-Test-Unitree-Go2-v0 \
    --load_run /path/to/checkpoint
```

**Sim2Real路径**:
```
IsaacLab训练 → IsaacLab验证 → 真实机器人部署
```

### 方案B: 导出JIT模型

从IsaacLab导出 TorchScript 模型用于部署：

```python
# 在IsaacLab训练脚本中
from isaaclab.utils.exporters import export_policy_as_jit

export_policy_as_jit(
    policy=policy,
    path="exported/policy.pt",
    normalizer=obs_normalizer
)
```

### 方案C: 创建匹配的MuJoCo模型 (需重新训练)

如果要使用MuJoCo，需要：

1. **模型对齐**
   - 从IsaacLab导出MuJoCo模型 (如果支持)
   - 或手动匹配运动学参数

2. **参数调整**
   - 匹配质量、惯性、摩擦系数
   - 调整PD增益以适应新物理引擎

3. **域适应**
   - 在MuJoCo中微调策略
   - 或使用域随机化训练

## 使用示例

### 基础推理

```python
from policy_inference import IsaacLabPolicy, create_observation
import numpy as np

# 加载策略
policy = IsaacLabPolicy(
    "/home/robot/work/LekiwiTest/UntrieeGo2/policy/Rough_Walk_policy.pt"
)

# 准备观测
obs = create_observation(
    base_lin_vel=np.zeros(3),
    base_ang_vel=np.zeros(3),
    projected_gravity=np.array([0, 0, -1]),
    velocity_command=np.array([0.5, 0, 0]),  # 前进0.5m/s
    joint_pos=np.zeros(12),
    joint_vel=np.zeros(12),
    last_action=np.zeros(12),
    height_scan=np.zeros(187),
)

# 获取动作
action = policy.get_action(obs)  # 返回12维关节位置偏移

# 应用到关节 (IsaacLab顺序 FL, FR, RL, RR)
target_positions = default_positions + action
```

## 关键经验总结

1. **环境一致性最重要**: 策略在哪个环境训练，就应该在哪个环境部署
2. **物理引擎差异显著**: 不同引擎的动力学特性难以完全匹配
3. **运动学必须对齐**: 关节顺序、限位、默认位置必须一致
4. **观测要准确**: 特别是投影重力等关键状态信息
5. **Sim2Real建议**: IsaacLab → 真实机器人，而非 IsaacLab → MuJoCo → 真实机器人

## 参考资源

- **IsaacLab配置**: `/home/robot/work/BiShe/IsaacLabBisHe/source/MyProject/MyProject/tasks/manager_based/WalkTest/walk_test_env_cfg.py`
- **Unitree配置**: `/home/robot/work/IsaacLab-main/source/isaaclab_assets/isaaclab_assets/robots/unitree.py`
- **Sim2Sim示例**: `/home/robot/work/IsaacLab-main/scripts/sim2sim_transfer`
- **Humanoid-Gym**: `/home/robot/work/humanoid-gym` (成功的Sim2Real框架)

## 下一步工作

如果需要继续MuJoCo部署：

1. 获取与IsaacLab完全匹配的MuJoCo模型
2. 验证物理参数（质量、惯性、摩擦）
3. 调整PD增益以稳定姿态
4. 考虑域适应训练
5. 或使用IsaacLab作为主要仿真环境

## 文件清单

```
code/
├── policy_inference.py      # 核心推理类
├── policy_config.py          # 配置参数
├── demo.py                   # 简单演示
└── SIM2SIM_MIGRATION_GUIDE.md  # 本文档
```

---

**结论**: 对于IsaacLab训练的策略，**强烈建议在IsaacLab中进行部署和验证**，然后再转移到真实机器人。跨物理引擎的Sim2Sim迁移需要大量额外工作来对齐模型参数。
