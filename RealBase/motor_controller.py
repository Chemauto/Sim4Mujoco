#!/usr/bin/env python3
"""
三全向轮底盘运动控制器
使用Feetech STS3215舵机驱动
轮子配置: 120°间隔 (轮子1:0°, 轮子2:120°, 轮子3:240°)
"""

import time
import numpy as np
from typing import Dict, Optional

from motors.feetech.feetech import FeetechMotorsBus, OperatingMode
from motors import Motor, MotorNormMode


class OmniWheelController:
    """3全向轮底盘控制器

    支持三轮全向轮的独立速度控制,使用逆运动学矩阵实现
    """

    # 电机ID配置 - 修改这里即可更改所有电机ID
    WHEEL_IDS = [13, 14, 15]

    def __init__(
        self,
        port: str = "/dev/ttyACM0",
        wheel_radius: float = 0.05,  # 轮子半径(米)
        robot_radius: float = 0.15,  # 机器人半径(米),从中心到轮子的距离
    ):
        """初始化底盘控制器

        Args:
            port: 串口端口
            wheel_radius: 轮子半径(米)
            robot_radius: 从机器人中心到轮子的距离(米)
        """
        self.port = port
        self.wheel_radius = wheel_radius
        self.robot_radius = robot_radius
        self.base_bus = None

        # 计算逆运动学矩阵
        # 轮子布置角度: 0°, 120°, 240°
        angles = [0, 2*np.pi/3, 4*np.pi/3]

        # 构建正运动学矩阵 F_matrix (3x3)
        # 将轮子角速度映射到机器人速度 [vx, vy, omega]
        F_matrix = self.wheel_radius * np.array([
            [np.cos(angles[0]), np.cos(angles[1]), np.cos(angles[2])],
            [np.sin(angles[0]), np.sin(angles[1]), np.sin(angles[2])],
            [1/self.robot_radius, 1/self.robot_radius, 1/self.robot_radius]
        ])

        # 逆运动学矩阵 (用于从机器人速度计算轮子速度)
        self.F_matrix_inv = np.linalg.inv(F_matrix)

        # 当前轮子控制值 (rad/s)
        self.wheel_velocities = np.array([0.0, 0.0, 0.0])

    def connect(self) -> bool:
        """连接底盘电机

        Returns:
            连接成功返回True,否则返回False
        """
        try:
            # 创建底盘电机总线
            self.base_bus = FeetechMotorsBus(
                port=self.port,
                motors={
                    "wheel_1": Motor(OmniWheelController.WHEEL_IDS[0], "sts3215", MotorNormMode.RANGE_M100_100),
                    "wheel_2": Motor(OmniWheelController.WHEEL_IDS[1], "sts3215", MotorNormMode.RANGE_M100_100),
                    "wheel_3": Motor(OmniWheelController.WHEEL_IDS[2], "sts3215", MotorNormMode.RANGE_M100_100),
                },
            )

            # 连接总线
            self.base_bus.connect()
            print(f"✓ 已连接到底盘电机总线: {self.port}")

            # 配置电机为速度控制模式
            with self.base_bus.torque_disabled():
                self.base_bus.configure_motors()
                for motor in self.base_bus.motors:
                    self.base_bus.write("Operating_Mode", motor, OperatingMode.VELOCITY.value)

            # 使能扭矩
            self.base_bus.enable_torque()

            return True

        except Exception as e:
            print(f"✗ 底盘连接失败: {e}")
            return False

    def disconnect(self):
        """断开底盘连接"""
        if self.base_bus is not None:
            try:
                # 停止所有电机
                self.stop()
                time.sleep(0.5)
                # 断开连接
                self.base_bus.disconnect(disable_torque=True)
                print(f"✓ 已断开底盘电机")
            except Exception as e:
                print(f"✗ 断开连接时出错: {e}")
            finally:
                self.base_bus = None

    def set_velocity(self, linear_speed: float, vx: float, vy: float, omega: float = 0.0) -> bool:
        """设置机器人速度 (归一化方向向量)

        Args:
            linear_speed: 速度大小 (m/s)
            vx: 机器人坐标系下x方向速度分量 (归一化, -1到1)
            vy: 机器人坐标系下y方向速度分量 (归一化, -1到1)
            omega: 旋转角速度 (rad/s), 正值=逆时针旋转, 负值=顺时针旋转

        Returns:
            成功返回True,失败返回False

        说明:
            vx, vy 是方向向量,会被归一化并乘以linear_speed
            omega 直接使用,不进行归一化

            例如:
                vx=1, vy=0 表示向前
                vx=0, vy=1 表示向左
                omega=1 表示逆时针旋转 1 rad/s
        """
        if self.base_bus is None:
            print("✗ 底盘未连接")
            return False

        try:
            # 归一化方向向量并乘以速度大小
            v_norm = np.sqrt(vx**2 + vy**2)
            if v_norm > 0:
                vx = vx / v_norm * linear_speed
                vy = vy / v_norm * linear_speed
            else:
                vx = 0
                vy = 0

            # 使用逆运动学矩阵计算轮子角速度
            robot_velocity = np.array([vx, vy, omega])
            wheel_angular_velocities = self.F_matrix_inv @ robot_velocity

            # 保存当前轮子速度
            self.wheel_velocities = wheel_angular_velocities

            # 应用到电机
            self._apply_wheel_velocities(wheel_angular_velocities)

            return True

        except Exception as e:
            print(f"✗ 设置速度失败: {e}")
            return False

    def set_velocity_raw(self, vx: float, vy: float, omega: float = 0.0) -> bool:
        """直接设置速度分量 (不归一化)

        Args:
            vx: x方向速度 (m/s)
            vy: y方向速度 (m/s)
            omega: 旋转角速度 (rad/s)

        Returns:
            成功返回True,失败返回False
        """
        if self.base_bus is None:
            print("✗ 底盘未连接")
            return False

        try:
            # 使用逆运动学矩阵计算轮子角速度
            robot_velocity = np.array([vx, vy, omega])
            wheel_angular_velocities = self.F_matrix_inv @ robot_velocity

            # 保存当前轮子速度
            self.wheel_velocities = wheel_angular_velocities

            # 应用到电机
            self._apply_wheel_velocities(wheel_angular_velocities)

            return True

        except Exception as e:
            print(f"✗ 设置速度失败: {e}")
            return False

    def _apply_wheel_velocities(self, wheel_angular_velocities: np.ndarray):
        """将轮子角速度应用到电机

        Args:
            wheel_angular_velocities: 3个轮子的角速度 (rad/s)
        """
        # 假设最大角速度为 10 rad/s
        max_angular_velocity = 10.0

        for i, wheel_angular_vel in enumerate(wheel_angular_velocities):
            # 转换为百分比速度
            speed_percent = min(100, max(0, abs(wheel_angular_vel) / max_angular_velocity * 100))

            # 确定方向
            direction = 1 if wheel_angular_vel >= 0 else -1

            # 转换为电机速度值
            velocity_value = int(direction * speed_percent * 10.23)  # 100 * 10.23 = 1023

            # 获取电机名称
            motor_name = f"wheel_{i + 1}"

            # 写入速度
            self.base_bus.write("Goal_Velocity", motor_name, velocity_value, normalize=False)

    def get_wheel_velocities(self) -> np.ndarray:
        """获取当前轮子控制速度

        Returns:
            numpy数组,包含3个轮子的角速度 (rad/s)
        """
        return self.wheel_velocities.copy()

    def stop(self) -> bool:
        """停止所有轮子

        Returns:
            成功返回True,失败返回False
        """
        if self.base_bus is None:
            print("✗ 底盘未连接")
            return False

        try:
            for wheel_id in self.WHEEL_IDS:
                wheel_index = self.WHEEL_IDS.index(wheel_id)
                motor_name = f"wheel_{wheel_index + 1}"
                self.base_bus.write("Goal_Velocity", motor_name, 0, normalize=False)

            self.wheel_velocities = np.array([0.0, 0.0, 0.0])
            return True

        except Exception as e:
            print(f"✗ 停止失败: {e}")
            return False

    def control_wheel(self, wheel_id: int, direction: int, speed: int) -> Dict:
        """控制单个轮子转动 (兼容旧接口)

        Args:
            wheel_id: 电机ID (使用 WHEEL_IDS)
            direction: 转动方向 (1=正转, -1=反转)
            speed: 转动速度 (0-100, 0表示停止)

        Returns:
            执行结果字典
        """
        if self.base_bus is None:
            return {
                "success": False,
                "message": "底盘未连接",
                "wheel_id": wheel_id
            }

        if wheel_id not in self.WHEEL_IDS:
            return {
                "success": False,
                "message": f"无效的轮子ID: {wheel_id}, 必须是{self.WHEEL_IDS[0]}, {self.WHEEL_IDS[1]}, 或{self.WHEEL_IDS[2]}",
                "wheel_id": wheel_id
            }

        if direction not in [1, -1]:
            return {
                "success": False,
                "message": f"无效的方向: {direction}, 必须是1或-1",
                "wheel_id": wheel_id
            }

        if not 0 <= speed <= 100:
            return {
                "success": False,
                "message": f"无效的速度: {speed}, 必须在0-100之间",
                "wheel_id": wheel_id
            }

        try:
            # 计算实际速度值 (考虑方向)
            actual_speed = direction * speed

            # 根据电机ID获取电机名称和索引
            wheel_index = self.WHEEL_IDS.index(wheel_id)
            motor_name = f"wheel_{wheel_index + 1}"

            # 设置速度
            velocity_value = int(actual_speed * 10.23)  # 100 * 10.23 = 1023

            self.base_bus.write("Goal_Velocity", motor_name, velocity_value, normalize=False)

            # 更新当前速度
            self.wheel_velocities[wheel_index] = actual_speed / 10.23

            direction_str = "正转" if direction > 0 else "反转"

            return {
                "success": True,
                "message": f"轮子{wheel_id} {direction_str}, 速度: {speed}%",
                "wheel_id": wheel_id,
                "direction": direction,
                "direction_str": direction_str,
                "speed": speed,
                "actual_speed": actual_speed
            }

        except Exception as e:
            return {
                "success": False,
                "message": f"控制失败: {str(e)}",
                "wheel_id": wheel_id
            }

    def stop_wheel(self, wheel_id: int) -> Dict:
        """停止指定轮子

        Args:
            wheel_id: 电机ID

        Returns:
            执行结果字典
        """
        return self.control_wheel(wheel_id, direction=1, speed=0)

    def get_wheel_speed(self, wheel_id: int) -> Dict:
        """获取指定轮子的当前速度

        Args:
            wheel_id: 电机ID

        Returns:
            速度信息字典
        """
        if wheel_id not in self.WHEEL_IDS:
            return {
                "success": False,
                "message": f"无效的轮子ID: {wheel_id}",
                "wheel_id": wheel_id
            }

        try:
            wheel_index = self.WHEEL_IDS.index(wheel_id)
            speed_rad_s = self.wheel_velocities[wheel_index]
            speed_percent = abs(speed_rad_s) / 10.0 * 100

            return {
                "success": True,
                "wheel_id": wheel_id,
                "speed": min(100, speed_percent),
                "direction": 1 if speed_rad_s >= 0 else -1,
                "direction_str": "正转" if speed_rad_s >= 0 else "反转"
            }

        except Exception as e:
            return {
                "success": False,
                "message": f"读取速度失败: {str(e)}",
                "wheel_id": wheel_id
            }

    def get_all_speeds(self) -> Dict:
        """获取所有轮子的当前速度

        Returns:
            所轮子速度信息
        """
        if self.base_bus is None:
            return {
                "success": False,
                "message": "底盘未连接"
            }

        try:
            speeds = {}
            for i, wheel_id in enumerate(self.WHEEL_IDS):
                speed_rad_s = self.wheel_velocities[i]
                speed_percent = abs(speed_rad_s) / 10.0 * 100

                speeds[f"wheel_{i + 1}"] = {
                    "wheel_id": wheel_id,
                    "speed": min(100, speed_percent),
                    "direction": 1 if speed_rad_s >= 0 else -1,
                    "direction_str": "正转" if speed_rad_s >= 0 else "反转"
                }

            return {
                "success": True,
                "speeds": speeds
            }

        except Exception as e:
            return {
                "success": False,
                "message": f"读取速度失败: {str(e)}"
            }


# 全局控制器实例
_motor_controller: Optional[OmniWheelController] = None


def get_motor_controller() -> OmniWheelController:
    """获取全局底盘控制器实例

    Returns:
        底盘控制器实例
    """
    global _motor_controller
    if _motor_controller is None:
        _motor_controller = OmniWheelController()
    return _motor_controller
