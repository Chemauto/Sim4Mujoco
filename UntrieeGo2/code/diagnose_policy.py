#!/usr/bin/env python3
"""
诊断策略输出 - 分析策略对不同速度命令的响应
"""
import os
os.environ['MUJOCO_GL'] = 'egl'

import numpy as np
import torch
from mujoco_deploy import MuJoCoDemo

def diagnose_policy():
    """诊断策略输出"""
    model_path = "/home/robot/work/LekiwiTest/UntrieeGo2/model/go2/scene.xml"
    policy_path = "/home/robot/work/LekiwiTest/UntrieeGo2/policy/Rough_Walk_policy4Mujoco.pt"

    demo = MuJoCoDemo(model_path, policy_path)
    demo.reset_to_initial_state()

    print("\n" + "="*60)
    print("策略诊断 - 测试不同速度命令的响应")
    print("="*60)

    # 测试不同的速度命令
    test_commands = [
        (0.0, 0.0, 0.0, "站立"),
        (0.3, 0.0, 0.0, "慢速前进"),
        (0.5, 0.0, 0.0, "中速前进"),
        (1.0, 0.0, 0.0, "快速前进"),
        (0.0, 0.0, 0.5, "旋转"),
    ]

    for vx, vy, wz, desc in test_commands:
        demo.vx, demo.vy, demo.wz = vx, vy, wz

        # 获取观测
        obs = demo.get_observation()

        # 获取动作
        action = demo.policy.get_action(obs)

        print(f"\n{desc} (vx={vx:.1f}, vy={vy:.1f}, wz={wz:.1f}):")
        print(f"  观测中的速度命令: [{obs[9]:.3f}, {obs[10]:.3f}, {obs[11]:.3f}]")
        print(f"  策略输出（未缩放）: [{action[0]:.3f}, {action[1]:.3f}, {action[2]:.3f}, ...]")
        print(f"  动作范围: [{action.min():.3f}, {action.max():.3f}]")
        print(f"  动作标准差: {action.std():.3f}")

        # 分析FL腿的动作
        print(f"  FL腿动作: [{action[0]:.3f}, {action[1]:.3f}, {action[2]:.3f}]")
        print(f"  FR腿动作: [{action[3]:.3f}, {action[4]:.3f}, {action[5]:.3f}]")

    print("\n" + "="*60)
    print("诊断完成")
    print("="*60)

    # 分析：如果策略输出变化很小，说明策略没有学会响应速度命令
    print("\n分析:")
    print("  - 如果不同速度命令的策略输出相似，说明策略没有学会")
    print("  - 建议增加训练迭代次数")
    print("  - 或者在MuJoCo中直接训练（Sim2Real）")

if __name__ == "__main__":
    diagnose_policy()
