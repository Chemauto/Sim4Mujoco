#!/usr/bin/env python3
"""
IsaacLab策略在MuJoCo中的演示
展示如何使用IsaacLab训练的策略控制Unitree Go2
"""
import numpy as np
from policy_inference import IsaacLabPolicy, create_observation


def main():
    """演示策略使用"""
    print("=" * 60)
    print("IsaacLab策略演示 - Template-Velocity-Test-Unitree-Go2-v0")
    print("=" * 60)

    # 策略路径
    policy_path = "/home/robot/work/LekiwiTest/UntrieeGo2/policy/Rough_Walk_policy.pt"

    # 加载策略
    print(f"\n[1] 加载策略...")
    policy = IsaacLabPolicy(policy_path, device='cpu')

    # 设置速度命令 - 让机器人前进
    vx, vy, wz = 0.5, 0.0, 0.0
    velocity_command = np.array([vx, vy, wz])
    print(f"[2] 设置速度命令: vx={vx} m/s, vy={vy} m/s, wz={wz} rad/s")

    # 初始化状态
    joint_pos = np.zeros(12)
    joint_vel = np.zeros(12)
    last_action = np.zeros(12)

    print(f"[3] 开始推理演示 (10步)...\n")

    # 推理演示
    for step in range(10):
        # 构建观测 (235维)
        obs = create_observation(
            base_lin_vel=np.array([0.1, 0.0, 0.0]),        # 基座线速度
            base_ang_vel=np.array([0.0, 0.0, 0.0]),        # 基座角速度
            projected_gravity=np.array([0.0, 0.0, -1.0]),  # 投影重力
            velocity_command=velocity_command,              # 速度命令
            joint_pos=joint_pos,                           # 关节位置
            joint_vel=joint_vel,                           # 关节速度
            last_action=last_action,                       # 上一个动作
            height_scan=np.zeros(187),                     # 高度扫描(可选)
        )

        # 获取动作 (12维关节位置偏移)
        action = policy.get_action(obs)

        # 打印结果
        print(f"步骤 {step}: 动作范围 [{action.min():.3f}, {action.max():.3f}]")

        # 更新状态 (模拟)
        # 在真实MuJoCo中: target_pos = current_pos + action
        last_action = action.copy()

    print(f"\n✓ 演示完成!")
    print(f"\n策略信息:")
    print(f"  - 训练迭代: 1499")
    print(f"  - 输入维度: 235 (观测空间)")
    print(f"  - 输出维度: 12 (关节动作)")
    print(f"  - 网络结构: [512, 256, 128]")


if __name__ == "__main__":
    main()
