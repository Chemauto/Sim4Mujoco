#!/usr/bin/env python3
"""
使用真实Xbox手柄远程控制底盘
在电脑上运行此程序，通过网络控制开发板上的底盘
"""

import sys
import time
import argparse
import socket
import pickle

try:
    import pygame
    from pygame import joystick
except ImportError:
    print("错误: 未安装pygame库")
    print("请运行: pip install pygame")
    sys.exit(1)

# --- 配置参数 ---
LINEAR_SPEED = 0.3  # 最大线速度 (m/s)
OMEGA_SPEED = 0.8  # 最大角速度 (rad/s)
JOYSTICK_DEADZONE = 0.1  # 手柄死区

# 速度档位设置
SPEED_MODES = [
    {"linear": 0.15, "omega": 0.4, "name": "低速"},
    {"linear": 0.30, "omega": 0.8, "name": "中速"},
    {"linear": 0.50, "omega": 1.2, "name": "高速"}
]


class XboxJoystickReader:
    """读取真实Xbox手柄状态"""

    def __init__(self, joystick_id=0):
        """初始化手柄

        Args:
            joystick_id: 手柄ID，默认为0（第一个连接的手柄）
        """
        # 初始化pygame
        pygame.init()
        pygame.joystick.init()

        # 检查是否有手柄连接
        if pygame.joystick.get_count() == 0:
            raise RuntimeError("未检测到任何手柄设备！请确保手柄已连接并重新插拔。")

        # 打开手柄
        self.js = pygame.joystick.Joystick(joystick_id)
        self.js.init()

        print(f"已连接手柄: {self.js.get_name()}")
        print(f"轴数量: {self.js.get_numaxes()}")
        print(f"按键数量: {self.js.get_numbuttons()}")
        print(f"方向键数量: {self.js.get_numhats()}")

        # 校准摇杆零点偏移
        self.axis_offset = []
        self._calibrate()

    def _calibrate(self):
        """校准摇杆零点，记录初始偏移"""
        print("\n正在校准手柄，请松开所有摇杆和按键...")
        time.sleep(0.5)  # 等待手柄稳定

        # 清空事件队列
        for _ in pygame.event.get():
            pass

        # 读取所有轴的初始值作为偏移
        num_axes = self.js.get_numaxes()
        self.axis_offset = []
        for i in range(num_axes):
            offset = self.js.get_axis(i)
            self.axis_offset.append(offset)

        print("校准完成!")
        print("轴偏移量:", [f"{x:.3f}" for x in self.axis_offset])

    def get_axis(self, axis_id: int) -> float:
        """获取轴值（已减去零点偏移）

        Args:
            axis_id: 轴ID (0-左摇杆X, 1-左摇杆Y, 2-右摇杆X, 3-右摇杆Y)

        Returns:
            轴值，范围 [-1.0, 1.0]
        """
        if axis_id < self.js.get_numaxes():
            # 轴值需要处理pygame事件队列才能更新
            for _ in pygame.event.get():
                pass
            raw_value = self.js.get_axis(axis_id)
            # 减去校准时的零点偏移
            offset = self.axis_offset[axis_id] if axis_id < len(self.axis_offset) else 0.0
            corrected_value = raw_value - offset
            # 限制在[-1, 1]范围内
            return max(-1.0, min(1.0, corrected_value))
        return 0.0

    def get_button(self, button_id: int) -> bool:
        """获取按键状态

        Args:
            button_id: 按键ID

        Returns:
            True表示按下，False表示未按下
        """
        if button_id < self.js.get_numbuttons():
            # 按键状态需要处理pygame事件队列才能更新
            for _ in pygame.event.get():
                pass
            return self.js.get_button(button_id)
        return False


class RemoteController:
    """通过网络远程控制底盘"""

    def __init__(self, host, port):
        """初始化远程控制器

        Args:
            host: 开发板IP地址
            port: 通信端口
        """
        self.host = host
        self.port = port
        self.sock = None
        self.connected = False

    def connect(self):
        """连接到开发板"""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(2.0)  # 2秒超时
            self.sock.connect((self.host, self.port))
            self.connected = True
            print(f"✓ 已连接到开发板 {self.host}:{self.port}")
            return True
        except Exception as e:
            print(f"✗ 连接失败: {e}")
            return False

    def send_velocity(self, vx, vy, omega):
        """发送速度指令

        Args:
            vx: x方向速度 (m/s)
            vy: y方向速度 (m/s)
            omega: 旋转角速度 (rad/s)
        """
        if not self.connected or self.sock is None:
            return False

        try:
            data = pickle.dumps({
                'type': 'velocity',
                'vx': vx,
                'vy': vy,
                'omega': omega
            })
            self.sock.sendall(data)
            return True
        except Exception as e:
            print(f"\n✗ 发送失败: {e}")
            self.connected = False
            return False

    def send_stop(self):
        """发送停止指令"""
        return self.send_velocity(0, 0, 0)

    def disconnect(self):
        """断开连接"""
        if self.sock:
            try:
                self.send_stop()
                self.sock.close()
            except:
                pass
            finally:
                self.sock = None
                self.connected = False


def main(host, port):
    """主函数"""
    print("=" * 60)
    print("Xbox手柄远程控制程序")
    print("=" * 60)
    print(f"开发板地址: {host}:{port}")

    # 创建Xbox手柄读取器
    try:
        joystick_reader = XboxJoystickReader()
    except RuntimeError as e:
        print(f"\n错误: {e}")
        print("\n请确保:")
        print("  - Xbox手柄已连接到电脑")
        print("  - 手柄驱动已正确安装")
        print("  - 已安装pygame库: pip install pygame")
        return

    print("\n控制映射:")
    print("  - 左摇杆 (上下): 前进/后退 (vx)")
    print("  - 左摇杆 (左右): 左移/右移 (vy)")
    print("  - 右摇杆 (左右): 原地旋转 (omega)")
    print("  - Xbox键(LS): 切换速度档位 (低/中/高)")
    print("\n速度档位:")
    for mode in SPEED_MODES:
        print(f"  - {mode['name']}模式: {mode['linear']} m/s, {mode['omega']} rad/s")
    print("\n按 Ctrl+C 退出程序")
    print("=" * 60)

    # 连接到开发板
    remote = RemoteController(host, port)
    print("\n正在连接开发板...")
    if not remote.connect():
        print("\n请确保:")
        print(f"  - 开发板IP地址正确: {host}")
        print("  - 开发板上已运行 joy_remote_server.py")
        return

    current_mode = 1  # 默认中速模式
    last_button_state = False

    try:
        while True:
            # 处理pygame事件
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    raise KeyboardInterrupt

            # 读取摇杆轴值
            # 左摇杆 Y 轴 (axis 1): 前进/后退, 上为-1, 下为1, 所以取反
            vx_raw = -joystick_reader.get_axis(1)
            # 左摇杆 X 轴 (axis 0): 左移/右移
            vy_raw = joystick_reader.get_axis(0)
            # 右摇杆 X 轴 (axis 3): 旋转, 取反使右摇杆右转为正
            omega_raw = -joystick_reader.get_axis(3)

            # 应用死区
            vx = vx_raw if abs(vx_raw) > JOYSTICK_DEADZONE else 0.0
            vy = vy_raw if abs(vy_raw) > JOYSTICK_DEADZONE else 0.0
            omega = omega_raw if abs(omega_raw) > JOYSTICK_DEADZONE else 0.0

            # 检测Xbox左摇杆按键(LS/按钮8)用于切换速度档位
            ls_button = joystick_reader.get_button(8)
            if ls_button and not last_button_state:
                current_mode = (current_mode + 1) % len(SPEED_MODES)
                mode = SPEED_MODES[current_mode]
                print(f"\n切换到{mode['name']}模式: {mode['linear']} m/s, {mode['omega']} rad/s")
            last_button_state = ls_button

            # 获取当前速度档位
            mode = SPEED_MODES[current_mode]

            # 计算实际速度
            final_vx = vx * mode['linear']
            final_vy = vy * mode['linear']
            final_omega = omega * mode['omega']

            # 发送速度指令
            if vx == 0.0 and vy == 0.0 and omega == 0.0:
                remote.send_stop()
                print(f"\r[{mode['name']}] 停止中                                      ", end="")
            else:
                remote.send_velocity(final_vx, final_vy, final_omega)
                print(f"\r[{mode['name']}] Joystick -> vx: {vx:+.2f}, vy: {vy:+.2f}, omega: {omega:+.2f} | "
                      f"Sending -> vx: {final_vx:+.2f}m/s, vy: {final_vy:+.2f}m/s, omega: {final_omega:+.2f}rad/s",
                      end="")

            # 循环延时
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n\n程序被中断, 正在停止机器人...")

    except Exception as e:
        print(f"\n发生错误: {e}")
        import traceback
        traceback.print_exc()

    finally:
        # 断开连接
        print("\n正在断开连接...")
        remote.send_stop()
        remote.disconnect()
        pygame.quit()
        print("程序结束")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="使用Xbox手柄远程控制3全向轮底盘")
    parser.add_argument(
        '--host',
        type=str,
        default='192.168.0.155',
        help='开发板IP地址'
    )
    parser.add_argument(
        '--port',
        type=int,
        default=9999,
        help='通信端口'
    )
    args = parser.parse_args()
    main(args.host, args.port)
