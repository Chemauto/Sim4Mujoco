"""
测试三麦克纳姆轮底盘运动学
用于验证运动学公式是否正确
"""

import os
import mujoco
import mujoco.viewer
from omni_controller import OmniWheelController


def test_movement(model_path):
    """测试基本运动"""
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    controller = OmniWheelController(model, data)

    print("\n" + "="*60)
    print("三麦克纳姆轮运动学测试")
    print("="*60)
    print("\n请测试以下运动并观察是否有角度偏转:")
    print("1. 前进/后退 - 应该直线运动,不偏转")
    print("2. 左移/右移 - 应该直线运动,不偏转")
    print("3. 原地旋转 - 应该在原地旋转,不位移")
    print("="*60 + "\n")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            try:
                print("\n选择测试:")
                print("  1 - 前进测试 (vx=1, vy=0)")
                print("  2 - 后退测试 (vx=-1, vy=0)")
                print("  3 - 左移测试 (vx=0, vy=1)")
                print("  4 - 右移测试 (vx=0, vy=-1)")
                print("  5 - 原地旋转测试 (omega=1)")
                print("  6 - 退出")

                choice = input("\n请选择 (1-6): ").strip()

                if choice == '6':
                    break

                duration = 2.0  # 每个测试2秒

                if choice == '1':
                    print("\n[前进测试] 机器人应该向前直线运动...")
                    controller.set_velocity(linear_speed=0.3, vx=1, vy=0, omega=0)
                elif choice == '2':
                    print("\n[后退测试] 机器人应该向后直线运动...")
                    controller.set_velocity(linear_speed=0.3, vx=-1, vy=0, omega=0)
                elif choice == '3':
                    print("\n[左移测试] 机器人应该向左直线运动(不偏转)...")
                    controller.set_velocity(linear_speed=0.3, vx=0, vy=1, omega=0)
                elif choice == '4':
                    print("\n[右移测试] 机器人应该向右直线运动(不偏转)...")
                    controller.set_velocity(linear_speed=0.3, vx=0, vy=-1, omega=0)
                elif choice == '5':
                    print("\n[旋转测试] 机器人应该在原地旋转(不位移)...")
                    controller.set_velocity(linear_speed=0, vx=0, vy=0, omega=0.5)
                else:
                    print("无效选择!")
                    continue

                # 执行测试
                import time
                start_time = time.time()
                start_pos = controller.get_robot_position().copy()

                while time.time() - start_time < duration and viewer.is_running():
                    controller.apply_control()
                    mujoco.mj_step(model, data)

                    # 打印进度
                    elapsed = time.time() - start_time
                    pos = controller.get_robot_position()
                    print(f"\r时间: {elapsed:.1f}s/{duration}s | 位置: [{pos[0]:.3f}, {pos[1]:.3f}]", end="")

                    viewer.sync()

                # 停止
                controller.stop()
                controller.apply_control()

                # 显示结果
                end_pos = controller.get_robot_position()
                displacement = end_pos - start_pos

                print(f"\n\n测试完成!")
                print(f"起始位置: [{start_pos[0]:.3f}, {start_pos[1]:.3f}]")
                print(f"结束位置: [{end_pos[0]:.3f}, {end_pos[1]:.3f}]")
                print(f"位移: [{displacement[0]:.3f}, {displacement[1]:.3f}]")

                # 检查是否有偏转
                if choice in ['1', '2']:  # 前进/后退
                    if abs(displacement[1]) > 0.05:  # y方向有位移
                        print("⚠️  警告: 检测到侧向偏转!")
                elif choice in ['3', '4']:  # 左移/右移
                    if abs(displacement[0]) > 0.05:  # x方向有位移
                        print("⚠️  警告: 检测到前后偏转!")
                elif choice == '5':  # 旋转
                    if np.linalg.norm(displacement[:2]) > 0.05:  # 有位移
                        print("⚠️  警告: 旋转时有位移!")

            except KeyboardInterrupt:
                print("\n\n测试中断")
                controller.stop()
                controller.apply_control()
                break
            except Exception as e:
                print(f"\n错误: {e}")
                import traceback
                traceback.print_exc()

    print("\n测试结束!")


if __name__ == "__main__":
    import numpy as np  # 确保导入了numpy

    # 使用相对路径自动适配不同电脑
    script_dir = os.path.dirname(os.path.abspath(__file__))
    model_path = os.path.join(script_dir, "../model/assets/scene.xml")

    test_movement(model_path)
