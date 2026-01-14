"""
全局导航控制器测试脚本
演示如何使用GlobalNavigator实现点到点移动
"""

import os
import mujoco
import mujoco.viewer
import time
import numpy as np
from omni_controller import OmniWheelController
from global_navigator import GlobalNavigator


def test_single_point():
    """测试: 移动到单个目标点"""
    print("\n" + "="*60)
    print("测试1: 移动到单个目标点")
    print("="*60)

    # 加载模型 (使用脚本相对路径,自动适配不同电脑)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    model_path = os.path.join(script_dir, "../model/assets/scene.xml")
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    # 创建控制器
    omni_controller = OmniWheelController(model, data)
    navigator = GlobalNavigator(model, data)

    # 设置目标: 移动到 (0.5, 0.5), 不控制姿态
    navigator.set_target(x=0.5, y=0.5, yaw=None)

    print("开始导航...")
    start_time = time.time()

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running() and time.time() - start_time < 20:
            # 计算控制输出
            vx_robot, vy_robot, omega = navigator.update(model.opt.timestep)

            # 应用控制
            omni_controller.set_velocity_raw(vx_robot, vy_robot, omega)
            omni_controller.apply_control()

            # 仿真步进
            mujoco.mj_step(model, data)
            viewer.sync()

            # 打印状态
            status = navigator.get_navigation_status()
            if navigator.is_navigating and int(time.time() * 10) % 10 == 0:
                print(f"位置: {status['current_position'][:2]}, "
                      f"误差: {status['position_error']*100:.2f}cm")

            if not navigator.is_navigating:
                print("到达目标!")
                time.sleep(1)
                break


def test_multi_point():
    """测试: 依次访问多个目标点"""
    print("\n" + "="*60)
    print("测试2: 依次访问多个目标点 (方形路径)")
    print("="*60)

    # 加载模型
    script_dir = os.path.dirname(os.path.abspath(__file__))
    model_path = os.path.join(script_dir, "../model/assets/scene.xml")
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    # 创建控制器
    omni_controller = OmniWheelController(model, data)
    navigator = GlobalNavigator(model, data)

    # 定义多个目标点 (方形路径)
    waypoints = [
        (0.5, 0.0, None),    # 右边
        (0.5, 0.5, None),    # 右上
        (0.0, 0.5, None),    # 上边
        (0.0, 0.0, None),    # 回到起点
    ]

    current_waypoint = 0
    waypoint_set = False
    total_waypoints = len(waypoints)

    print(f"将访问 {total_waypoints} 个目标点")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            # 设置下一个目标点
            if not waypoint_set and current_waypoint < total_waypoints:
                x, y, yaw = waypoints[current_waypoint]
                navigator.set_target(x, y, 0, yaw)
                waypoint_set = True
                print(f"\n前往目标点 {current_waypoint + 1}/{total_waypoints}: ({x}, {y})")

            # 计算控制输出
            vx_robot, vy_robot, omega = navigator.update(model.opt.timestep)

            # 应用控制
            omni_controller.set_velocity_raw(vx_robot, vy_robot, omega)
            omni_controller.apply_control()

            # 仿真步进
            mujoco.mj_step(model, data)
            viewer.sync()

            # 打印状态
            if navigator.is_navigating and int(time.time() * 10) % 10 == 0:
                status = navigator.get_navigation_status()
                print(f"位置: {status['current_position'][:2]}, "
                      f"误差: {status['position_error']*100:.2f}cm")

            # 检查是否到达当前目标
            if waypoint_set and not navigator.is_navigating:
                waypoint_set = False
                current_waypoint += 1
                print(f"到达目标点 {current_waypoint}!")

                if current_waypoint >= total_waypoints:
                    print("\n所有目标点已访问完成!")
                    time.sleep(1)
                    break


def test_with_orientation():
    """测试: 移动到目标点并同时控制姿态"""
    print("\n" + "="*60)
    print("测试3: 移动到目标点并控制姿态")
    print("="*60)

    # 加载模型
    script_dir = os.path.dirname(os.path.abspath(__file__))
    model_path = os.path.join(script_dir, "../model/assets/scene.xml")
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    # 创建控制器
    omni_controller = OmniWheelController(model, data)
    navigator = GlobalNavigator(model, data)

    # 定义目标点 (位置 + 姿态)
    waypoints = [
        (0.5, 0.0, 0.0),      # 右边, 朝向+x
        (0.5, 0.5, np.pi/2),  # 右上, 朝向+y
        (0.0, 0.5, np.pi),    # 上边, 朝向-x
        (0.0, 0.0, -np.pi/2), # 起点, 朝向-y
    ]

    current_waypoint = 0
    waypoint_set = False
    total_waypoints = len(waypoints)

    print(f"将访问 {total_waypoints} 个目标点 (每个都包含姿态控制)")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            # 设置下一个目标点
            if not waypoint_set and current_waypoint < total_waypoints:
                x, y, yaw = waypoints[current_waypoint]
                navigator.set_target(x, y, 0, yaw)
                waypoint_set = True
                yaw_deg = yaw * 180 / np.pi
                print(f"\n前往目标点 {current_waypoint + 1}/{total_waypoints}: "
                      f"位置({x}, {y}), 姿态{yaw_deg:.1f}°")

            # 计算控制输出
            vx_robot, vy_robot, omega = navigator.update(model.opt.timestep)

            # 应用控制
            omni_controller.set_velocity_raw(vx_robot, vy_robot, omega)
            omni_controller.apply_control()

            # 仿真步进
            mujoco.mj_step(model, data)
            viewer.sync()

            # 打印状态
            if navigator.is_navigating and int(time.time() * 10) % 10 == 0:
                status = navigator.get_navigation_status()
                pos_error = status['position_error'] * 100
                yaw_error = abs(status.get('yaw_error', 0)) * 180 / np.pi
                print(f"位置误差: {pos_error:.2f}cm, 姿态误差: {yaw_error:.2f}°")

            # 检查是否到达当前目标
            if waypoint_set and not navigator.is_navigating:
                waypoint_set = False
                current_waypoint += 1
                print(f"到达目标点 {current_waypoint}!")

                if current_waypoint >= total_waypoints:
                    print("\n所有目标点已访问完成!")
                    time.sleep(1)
                    break


def test_manual_input():
    """测试: 手动输入目标坐标"""
    print("\n" + "="*60)
    print("测试4: 手动输入目标坐标")
    print("="*60)

    # 加载模型
    script_dir = os.path.dirname(os.path.abspath(__file__))
    model_path = os.path.join(script_dir, "../model/assets/scene.xml")
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    # 创建控制器
    omni_controller = OmniWheelController(model, data)
    navigator = GlobalNavigator(model, data)

    print("\n使用说明:")
    print("  输入格式: x y [yaw]")
    print("  例如: 0.5 0.3         -> 移动到(0.5, 0.3), 不控制姿态")
    print("  例如: 0.5 0.3 1.57    -> 移动到(0.5, 0.3), 朝向90度")
    print("  输入 'q' 退出")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            if not navigator.is_navigating:
                # 等待用户输入
                try:
                    user_input = input("\n请输入目标坐标 (x y [yaw]): ").strip()
                    if user_input.lower() == 'q':
                        break

                    parts = user_input.split()
                    if len(parts) >= 2:
                        x = float(parts[0])
                        y = float(parts[1])
                        yaw = float(parts[2]) if len(parts) > 2 else None
                        navigator.set_target(x, y, 0, yaw)
                except (ValueError, IndexError):
                    print("输入格式错误,请重新输入")
                    continue
                except KeyboardInterrupt:
                    break

            # 计算控制输出
            vx_robot, vy_robot, omega = navigator.update(model.opt.timestep)

            # 应用控制
            omni_controller.set_velocity_raw(vx_robot, vy_robot, omega)
            omni_controller.apply_control()

            # 仿真步进
            mujoco.mj_step(model, data)
            viewer.sync()

            # 打印状态
            if navigator.is_navigating and int(time.time() * 10) % 10 == 0:
                status = navigator.get_navigation_status()
                pos_error = status['position_error'] * 100
                if 'yaw_error' in status:
                    yaw_error = abs(status['yaw_error']) * 180 / np.pi
                    print(f"位置误差: {pos_error:.2f}cm, 姿态误差: {yaw_error:.2f}°")
                else:
                    print(f"位置误差: {pos_error:.2f}cm")


def main():
    """主函数: 选择测试模式"""
    print("="*60)
    print("全局导航控制器测试程序")
    print("="*60)

    print("\n请选择测试模式:")
    print("  1 - 单点移动 (移动到一个目标点)")
    print("  2 - 多点移动 (方形路径)")
    print("  3 - 带姿态控制 (位置+姿态)")
    print("  4 - 手动输入 (自定义坐标)")

    try:
        choice = input("\n请输入选择 (1-4): ").strip()

        if choice == '1':
            test_single_point()
        elif choice == '2':
            test_multi_point()
        elif choice == '3':
            test_with_orientation()
        elif choice == '4':
            test_manual_input()
        else:
            print("无效的选择!")
    except KeyboardInterrupt:
        print("\n\n程序已退出")


if __name__ == "__main__":
    main()
