#!/usr/bin/env python3
"""底盘控制示例

演示如何使用motor_controller模块控制3全向轮底盘
"""

from motor_controller import OmniWheelController


def main():
    """主函数"""
    # 创建底盘控制器
    # 注意: 根据你的实际情况修改端口
    base = OmniWheelController(
        port="/dev/ttyACM0",  # 修改为你的实际端口
        wheel_radius=0.05,     # 轮子半径5cm
        robot_radius=0.15      # 机器人半径15cm
    )

    print("=" * 50)
    print("3全向轮底盘控制示例")
    print("=" * 50)

    # 连接底盘
    print("\n1. 连接底盘...")
    if not base.connect():
        print("连接失败,请检查:")
        print("  - 串口端口是否正确")
        print("  - 电机是否已连接")
        print(f"  - 电机ID是否正确({base.WHEEL_IDS[0]},{base.WHEEL_IDS[1]},{base.WHEEL_IDS[2]})")
        return

    try:
        # 示例1: 控制单个轮子
        print("\n" + "=" * 50)
        print("示例1: 控制单个轮子")
        print("=" * 50)

        print("\n轮子1正转,速度50%...")
        result = base.control_wheel(wheel_id=base.WHEEL_IDS[0], direction=1, speed=50)
        print(f"  结果: {result['message']}")

        import time
        time.sleep(2)

        print("\n轮子1反转,速度30%...")
        result = base.control_wheel(wheel_id=base.WHEEL_IDS[0], direction=-1, speed=30)
        print(f"  结果: {result['message']}")

        time.sleep(2)

        print("\n停止轮子1...")
        base.stop_wheel(base.WHEEL_IDS[0])
        print("  轮子1已停止")

        # 示例2: 同时控制多个轮子
        print("\n" + "=" * 50)
        print("示例2: 同时控制多个轮子")
        print("=" * 50)

        print("\n所有轮子正转,速度40%...")
        for wheel_id in base.WHEEL_IDS:
            result = base.control_wheel(wheel_id=wheel_id, direction=1, speed=40)
            print(f"  {result['message']}")

        time.sleep(2)

        print("\n停止所有轮子...")
        result = base.stop_all()
        print(f"  {result['message']}")

        # 示例3: 查询轮子速度
        print("\n" + "=" * 50)
        print("示例3: 查询轮子速度")
        print("=" * 50)

        # 先设置一些速度
        base.control_wheel(base.WHEEL_IDS[0], 1, 60)
        base.control_wheel(base.WHEEL_IDS[1], -1, 40)
        base.control_wheel(base.WHEEL_IDS[2], 1, 50)

        print("\n当前各轮子状态:")
        for wheel_id in base.WHEEL_IDS:
            result = base.get_wheel_speed(wheel_id)
            if result['success']:
                print(f"  轮子{wheel_id}: {result['direction_str']}, 速度={result['speed']}%")

        # 示例4: 向量控制(高级功能)
        print("\n" + "=" * 50)
        print("示例4: 向量控制(底盘整体运动)")
        print("=" * 50)

        print("\n前进 (vx=0.1m/s)...")
        result = base.move_vector(vx=0.1, vy=0.0, omega=0.0)
        if result['success']:
            print(f"  {result['message']}")
            print("  各轮子速度分配:")
            for wheel, speed_info in result['wheel_speeds'].items():
                dir_str = "正转" if speed_info['direction'] > 0 else "反转"
                print(f"    {wheel}: {dir_str}, 速度={speed_info['speed']}%")

        time.sleep(2)

        print("\n横移 (vy=0.1m/s)...")
        result = base.move_vector(vx=0.0, vy=0.1, omega=0.0)
        if result['success']:
            print(f"  {result['message']}")
            print("  各轮子速度分配:")
            for wheel, speed_info in result['wheel_speeds'].items():
                dir_str = "正转" if speed_info['direction'] > 0 else "反转"
                print(f"    {wheel}: {dir_str}, 速度={speed_info['speed']}%")

        time.sleep(2)

        print("\n原地旋转 (omega=0.5rad/s)...")
        result = base.move_vector(vx=0.0, vy=0.0, omega=0.5)
        if result['success']:
            print(f"  {result['message']}")
            print("  各轮子速度分配:")
            for wheel, speed_info in result['wheel_speeds'].items():
                dir_str = "正转" if speed_info['direction'] > 0 else "反转"
                print(f"    {wheel}: {dir_str}, 速度={speed_info['speed']}%")

        time.sleep(2)

        # 最终停止
        print("\n" + "=" * 50)
        print("停止所有轮子...")
        base.stop_all()
        print("=" * 50)

    except KeyboardInterrupt:
        print("\n\n用户中断,停止所有轮子...")
        base.stop_all()

    except Exception as e:
        print(f"\n\n发生错误: {e}")
        base.stop_all()

    finally:
        # 断开连接
        print("\n断开底盘连接...")
        base.disconnect()
        print("程序结束")


def test_single_wheel():
    """测试单个轮子控制"""
    base = OmniWheelController(port="/dev/ttyACM1")

    if base.connect():
        print("连接成功! 开始测试...")

        try:
            while True:
                print("\n请输入控制参数 (或输入 'q' 退出):")
                wheel_id = int(input("轮子ID (1/2/3): "))
                if wheel_id not in [1, 2, 3]:
                    print("无效的轮子ID!")
                    continue

                direction = int(input("方向 (1=正转, -1=反转): "))
                if direction not in [1, -1]:
                    print("无效的方向!")
                    continue

                speed = int(input("速度 (0-100): "))
                if not 0 <= speed <= 100:
                    print("无效的速度!")
                    continue

                result = base.control_wheel(wheel_id, direction, speed)
                print(f"\n执行结果: {result['message']}")

        except KeyboardInterrupt:
            print("\n停止所有轮子...")
            base.stop_all()
        finally:
            base.disconnect()


if __name__ == "__main__":
    # 取消下面一行的注释来运行交互式测试
    # test_single_wheel()

    # 运行示例程序
    main()
