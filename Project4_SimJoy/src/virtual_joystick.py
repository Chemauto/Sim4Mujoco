"""
虚拟手柄核心类
提供与pygame.joystick.Joystick兼容的接口
"""
from .joystick_state import JoystickState


class VirtualJoystick:
    """
    虚拟手柄类
    提供与pygame.joystick.Joystick相同的API接口
    可以作为drop-in replacement使用
    """

    def __init__(self, joy_id: int = 0):
        """
        初始化虚拟手柄

        Args:
            joy_id: 手柄ID（用于兼容pygame）
        """
        self._id = joy_id
        self._state = JoystickState()
        self._name = "Virtual Xbox Controller"

    def init(self):
        """初始化手柄（pygame兼容接口）"""
        return True

    def quit(self):
        """退出手柄（pygame兼容接口）"""
        pass

    def get_init(self):
        """获取初始化状态（pygame兼容接口）"""
        return True

    def get_id(self) -> int:
        """获取手柄ID"""
        return self._id

    def get_instance_id(self) -> int:
        """获取实例ID（pygame兼容接口）"""
        return self._id

    def get_guid(self) -> str:
        """获取手柄GUID（pygame兼容接口）"""
        return "virtual-xbox-controller-0000"

    def get_name(self) -> str:
        """获取手柄名称"""
        return self._name

    # ========== 按键接口 ==========
    def get_numbuttons(self) -> int:
        """获取按键数量"""
        return 8

    def get_button(self, button_id: int) -> bool:
        """
        获取按键状态

        Args:
            button_id: 按键ID (0-7)
                0: A
                1: B
                2: X
                3: Y
                4: LB
                5: RB
                6: SELECT
                7: START

        Returns:
            True表示按下，False表示未按下
        """
        return self._state.get_button(button_id)

    def set_button(self, button_id: int, value: bool):
        """
        设置按键状态（由UI调用）

        Args:
            button_id: 按键ID
            value: 按键状态
        """
        self._state.set_button(button_id, value)

    # ========== 轴接口 ==========
    def get_numaxes(self) -> int:
        """获取轴数量"""
        return 6

    def get_axis(self, axis_id: int) -> float:
        """
        获取轴值

        Args:
            axis_id: 轴ID (0-5)
                0: LX (左摇杆X轴) [-1, 1]
                1: LY (左摇杆Y轴) [-1, 1]
                2: LT (左扳机) [0, 1]
                3: RX (右摇杆X轴) [-1, 1]
                4: RY (右摇杆Y轴) [-1, 1]
                5: RT (右扳机) [0, 1]

        Returns:
            轴值（浮点数）
        """
        return self._state.get_axis(axis_id)

    def set_axis(self, axis_id: int, value: float):
        """
        设置轴值（由UI调用）

        Args:
            axis_id: 轴ID
            value: 轴值
        """
        self._state.set_axis(axis_id, value)

    # ========== 方向键接口 ==========
    def get_numhats(self) -> int:
        """获取方向键数量"""
        return 1

    def get_hat(self, hat_id: int = 0) -> tuple:
        """
        获取方向键状态

        Args:
            hat_id: 方向键ID

        Returns:
            元组 (x, y)
            x: -1(左), 0(中), 1(右)
            y: -1(下), 0(中), 1(上)
        """
        return self._state.get_hat(hat_id)

    def set_hat(self, hat_id: int, value: tuple):
        """
        设置方向键状态（由UI调用）

        Args:
            hat_id: 方向键ID
            value: (x, y) 元组
        """
        self._state.set_hat(hat_id, value)

    # ========== 状态管理 ==========
    def get_state(self) -> JoystickState:
        """
        获取完整的状态对象

        Returns:
            JoystickState对象
        """
        return self._state

    def reset(self):
        """重置所有状态"""
        self._state.reset()

    def __str__(self) -> str:
        """字符串表示"""
        return f"VirtualJoystick(id={self._id}, name='{self._name}')"

    def __repr__(self) -> str:
        """对象表示"""
        return self.__str__()
