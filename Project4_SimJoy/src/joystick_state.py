"""
手柄状态管理类
用于维护虚拟手柄的所有状态数据
"""
from typing import Dict, Tuple
from dataclasses import dataclass
from enum import Enum


class Button(Enum):
    """按键枚举"""
    A = 0
    B = 1
    X = 2
    Y = 3
    LB = 4
    RB = 5
    SELECT = 6
    START = 7


class Axis(Enum):
    """轴枚举"""
    LX = 0  # 左摇杆X轴
    LY = 1  # 左摇杆Y轴
    LT = 2  # 左扳机
    RX = 3  # 右摇杆X轴
    RY = 4  # 右摇杆Y轴
    RT = 5  # 右扳机


@dataclass
class JoystickState:
    """手柄状态数据类"""

    def __init__(self):
        # 按键状态 (button_id -> 状态)
        self.buttons: Dict[int, bool] = {
            0: False,  # A
            1: False,  # B
            2: False,  # X
            3: False,  # Y
            4: False,  # LB
            5: False,  # RB
            6: False,  # SELECT
            7: False,  # START
        }

        # 轴值 (axis_id -> 值)
        self.axes: Dict[int, float] = {
            0: 0.0,  # LX [-1, 1]
            1: 0.0,  # LY [-1, 1]
            2: 0.0,  # LT [0, 1]
            3: 0.0,  # RX [-1, 1]
            4: 0.0,  # RY [-1, 1]
            5: 0.0,  # RT [0, 1]
        }

        # 方向键状态 (hat_id -> (x, y))
        # x: -1(左), 0(中), 1(右)
        # y: -1(下), 0(中), 1(上)
        self.hats: Dict[int, Tuple[int, int]] = {
            0: (0, 0)  # D-PAD
        }

    def set_button(self, button_id: int, value: bool):
        """设置按键状态"""
        if button_id in self.buttons:
            self.buttons[button_id] = value

    def get_button(self, button_id: int) -> bool:
        """获取按键状态"""
        return self.buttons.get(button_id, False)

    def set_axis(self, axis_id: int, value: float):
        """设置轴值"""
        if axis_id in self.axes:
            # 限制值在有效范围内
            if axis_id in [2, 5]:  # LT, RT
                self.axes[axis_id] = max(0.0, min(1.0, value))
            else:  # 摇杆轴
                self.axes[axis_id] = max(-1.0, min(1.0, value))

    def get_axis(self, axis_id: int) -> float:
        """获取轴值"""
        return self.axes.get(axis_id, 0.0)

    def set_hat(self, hat_id: int, value: Tuple[int, int]):
        """设置方向键状态"""
        if hat_id in self.hats:
            self.hats[hat_id] = value

    def get_hat(self, hat_id: int) -> Tuple[int, int]:
        """获取方向键状态"""
        return self.hats.get(hat_id, (0, 0))

    def reset(self):
        """重置所有状态"""
        for button_id in self.buttons:
            self.buttons[button_id] = False

        for axis_id in self.axes:
            self.axes[axis_id] = 0.0

        for hat_id in self.hats:
            self.hats[hat_id] = (0, 0)

    def __str__(self) -> str:
        """返回状态字符串（用于调试）"""
        output = []
        output.append("=== Joystick State ===")
        output.append("\nButtons:")
        for btn_id, pressed in self.buttons.items():
            if pressed:
                output.append(f"  Button {btn_id}: PRESSED")

        output.append("\nAxes:")
        for axis_id, value in self.axes.items():
            if abs(value) > 0.01:
                output.append(f"  Axis {axis_id}: {value:.3f}")

        output.append("\nHats:")
        for hat_id, (x, y) in self.hats.items():
            if x != 0 or y != 0:
                output.append(f"  Hat {hat_id}: ({x}, {y})")

        return "\n".join(output)
