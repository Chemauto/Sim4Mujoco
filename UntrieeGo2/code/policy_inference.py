#!/usr/bin/env python3
"""
IsaacLab策略推理器 - 用于MuJoCo
在MuJoCo环境中加载和运行IsaacLab训练的策略
"""
import torch
import numpy as np
from typing import Dict, Tuple


class IsaacLabPolicy:
    """IsaacLab策略推理类"""

    def __init__(self, policy_path: str, device='cpu'):
        """
        初始化策略

        Args:
            policy_path: 策略权重文件路径 (.pt或.pth)
            device: 运行设备 ('cpu' 或 'cuda')
        """
        self.device = torch.device(device)
        self.policy_path = policy_path

        # 加载checkpoint
        checkpoint = torch.load(policy_path, map_location=self.device)

        # 提取actor权重
        actor_state_dict = {}
        for key, value in checkpoint['model_state_dict'].items():
            if key.startswith('actor.'):
                new_key = key[6:]  # 移除'actor.'前缀
                actor_state_dict[new_key] = value

        # 创建网络结构
        # 根据配置: [512, 256, 128] 三层隐藏层
        self.layers = []
        layer_configs = [
            (235, 512),   # 输入层
            (512, 256),   # 隐藏层1
            (256, 128),   # 隐藏层2
            (128, 12),    # 输出层 (12个关节)
        ]

        current_dim = 235
        for i, (in_dim, out_dim) in enumerate(layer_configs):
            # 加载权重和偏置
            weight_key = f'{i*2}.weight'
            bias_key = f'{i*2}.bias'

            if weight_key in actor_state_dict and bias_key in actor_state_dict:
                weight = actor_state_dict[weight_key].to(self.device)
                bias = actor_state_dict[bias_key].to(self.device)

                self.layers.append({
                    'weight': weight,
                    'bias': bias,
                    'activation': 'elu' if i < len(layer_configs) - 1 else None
                })
                current_dim = out_dim
            else:
                raise ValueError(f"找不到层 {i} 的权重: {weight_key}, {bias_key}")

        # 加载标准差 (用于动作缩放)
        if 'std' in checkpoint['model_state_dict']:
            self.std = checkpoint['model_state_dict']['std'].to(self.device)
        else:
            self.std = None

        print(f"策略加载成功: {len(self.layers)} 层")
        print(f"输入维度: 235, 输出维度: 12")
        print(f"设备: {self.device}")

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        """
        前向传播

        Args:
            obs: 观测 [batch_size, obs_dim] 或 [obs_dim]

        Returns:
            action: 动作 [batch_size, action_dim] 或 [action_dim]
        """
        # 确保输入是2D张量
        if obs.dim() == 1:
            obs = obs.unsqueeze(0)

        x = obs.to(self.device)

        # 通过各层前向传播
        for i, layer in enumerate(self.layers):
            x = torch.mm(x, layer['weight'].t()) + layer['bias']

            # 应用激活函数 (除了最后一层)
            if layer['activation'] == 'elu':
                x = torch.nn.functional.elu(x)

        return x.squeeze(0) if x.shape[0] == 1 else x

    def get_action(self, obs: np.ndarray) -> np.ndarray:
        """
        从numpy观测获取动作

        Args:
            obs: 观测数组 [obs_dim]

        Returns:
            action: 动作数组 [action_dim]
        """
        obs_tensor = torch.from_numpy(obs).float()

        with torch.no_grad():
            action_tensor = self.forward(obs_tensor)

        action = action_tensor.cpu().numpy()

        # 动作是关节位置偏移, 需要缩放
        # IsaacLab中使用 scale=0.25
        action = action * 0.25

        return action

    def reset(self):
        """重置策略状态 (对于RNN策略)"""
        pass


def create_observation(
    base_lin_vel: np.ndarray,        # [3] - 基座线速度
    base_ang_vel: np.ndarray,        # [3] - 基座角速度
    projected_gravity: np.ndarray,   # [3] - 投影重力向量
    velocity_command: np.ndarray,    # [3] - 速度命令 (vx, vy, wz)
    joint_pos: np.ndarray,           # [12] - 关节位置
    joint_vel: np.ndarray,           # [12] - 关节速度
    last_action: np.ndarray,         # [12] - 上一个动作
    height_scan: np.ndarray = None,  # [187] - 高度扫描
) -> np.ndarray:
    """
    构建观测向量

    按照IsaacLab配置顺序:
    1. base_lin_vel: [3]
    2. base_ang_vel: [3]
    3. projected_gravity: [3]
    4. velocity_commands: [3]
    5. joint_pos_rel: [12]
    6. joint_vel_rel: [12]
    7. actions: [12]
    8. height_scan: [187]

    总计: 3+3+3+3+12+12+12+187 = 235
    """
    obs_components = []

    # 1. 基座线速度 [3]
    obs_components.append(base_lin_vel)

    # 2. 基座角速度 [3]
    obs_components.append(base_ang_vel)

    # 3. 投影重力 [3]
    obs_components.append(projected_gravity)

    # 4. 速度命令 [3]
    obs_components.append(velocity_command)

    # 5. 关节位置 (相对于默认位置) [12]
    obs_components.append(joint_pos)

    # 6. 关节速度 [12]
    obs_components.append(joint_vel)

    # 7. 上一个动作 [12]
    obs_components.append(last_action)

    # 8. 高度扫描 [187]
    if height_scan is not None:
        obs_components.append(height_scan)
    else:
        # 如果没有高度扫描, 使用零
        obs_components.append(np.zeros(187))

    # 拼接所有组件
    obs = np.concatenate(obs_components)

    # 验证维度
    assert obs.shape[0] == 235, f"观测维度错误: {obs.shape[0]}, 期望 235"

    return obs


if __name__ == "__main__":
    # 测试代码
    import sys

    if len(sys.argv) < 2:
        print("用法: python policy_inference.py <policy_path>")
        sys.exit(1)

    policy_path = sys.argv[1]

    # 加载策略
    policy = IsaacLabPolicy(policy_path, device='cpu')

    # 创建随机观测
    obs = np.random.randn(235)

    # 获取动作
    action = policy.get_action(obs)

    print(f"\n测试:")
    print(f"  输入观测形状: {obs.shape}")
    print(f"  输出动作形状: {action.shape}")
    print(f"  动作值: {action}")
