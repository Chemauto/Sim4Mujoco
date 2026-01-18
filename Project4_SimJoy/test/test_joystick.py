"""
单元测试脚本
测试虚拟手柄的所有功能
"""
import sys
import time
import os

# 添加src目录到路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from src.virtual_joystick import VirtualJoystick
from src.joystick_state import JoystickState, Button, Axis


def test_virtual_joystick():
    """测试虚拟手柄的基本功能"""
    print("=" * 50)
    print("Testing VirtualJoystick...")
    print("=" * 50)

    # 创建虚拟手柄
    joystick = VirtualJoystick(0)

    # 测试初始化
    print("\n1. Testing initialization...")
    print(f"   Name: {joystick.get_name()}")
    print(f"   ID: {joystick.get_id()}")
    print(f"   Num buttons: {joystick.get_numbuttons()}")
    print(f"   Num axes: {joystick.get_numaxes()}")
    print(f"   Num hats: {joystick.get_numhats()}")

    # 测试按键
    print("\n2. Testing buttons...")
    for i in range(8):
        assert joystick.get_button(i) == False, f"Button {i} should be False initially"

    # 模拟按下A键
    joystick.set_button(0, True)
    assert joystick.get_button(0) == True, "Button A should be True"
    print("   Button A press: PASS")

    # 释放A键
    joystick.set_button(0, False)
    assert joystick.get_button(0) == False, "Button A should be False"
    print("   Button A release: PASS")

    # 测试所有按键
    button_names = ["A", "B", "X", "Y", "LB", "RB", "SELECT", "START"]
    for i, name in enumerate(button_names):
        joystick.set_button(i, True)
        assert joystick.get_button(i) == True, f"Button {name} should be True"
        joystick.set_button(i, False)
        assert joystick.get_button(i) == False, f"Button {name} should be False"
    print("   All buttons test: PASS")

    # 测试轴
    print("\n3. Testing axes...")
    for i in range(6):
        assert joystick.get_axis(i) == 0.0, f"Axis {i} should be 0.0 initially"

    # 测试摇杆轴
    joystick.set_axis(0, 0.5)  # LX
    assert abs(joystick.get_axis(0) - 0.5) < 0.01, "Axis LX should be 0.5"
    print("   Axis LX set to 0.5: PASS")

    joystick.set_axis(0, -1.0)  # LX
    assert abs(joystick.get_axis(0) - (-1.0)) < 0.01, "Axis LX should be -1.0"
    print("   Axis LX set to -1.0: PASS")

    # 测试扳机轴
    joystick.set_axis(2, 0.7)  # LT
    assert abs(joystick.get_axis(2) - 0.7) < 0.01, "Axis LT should be 0.7"
    print("   Axis LT set to 0.7: PASS")

    # 测试轴值限制
    joystick.set_axis(0, 2.0)  # 超出范围
    assert joystick.get_axis(0) == 1.0, "Axis should be clamped to 1.0"
    print("   Axis clamping test: PASS")

    # 重置轴
    joystick.set_axis(0, 0.0)
    joystick.set_axis(2, 0.0)

    # 测试方向键
    print("\n4. Testing hats...")
    assert joystick.get_hat(0) == (0, 0), "Hat should be centered initially"

    # 测试上方向
    joystick.set_hat(0, (0, 1))
    assert joystick.get_hat(0) == (0, 1), "Hat should be up"
    print("   Hat up: PASS")

    # 测试下方向
    joystick.set_hat(0, (0, -1))
    assert joystick.get_hat(0) == (0, -1), "Hat should be down"
    print("   Hat down: PASS")

    # 测试左方向
    joystick.set_hat(0, (-1, 0))
    assert joystick.get_hat(0) == (-1, 0), "Hat should be left"
    print("   Hat left: PASS")

    # 测试右方向
    joystick.set_hat(0, (1, 0))
    assert joystick.get_hat(0) == (1, 0), "Hat should be right"
    print("   Hat right: PASS")

    # 测试对角线方向
    joystick.set_hat(0, (1, 1))
    assert joystick.get_hat(0) == (1, 1), "Hat should be up-right"
    print("   Hat diagonal: PASS")

    # 重置
    joystick.set_hat(0, (0, 0))

    # 测试重置功能
    print("\n5. Testing reset...")
    joystick.set_button(0, True)
    joystick.set_axis(0, 0.5)
    joystick.set_hat(0, (1, 0))
    joystick.reset()
    assert joystick.get_button(0) == False, "Button should be False after reset"
    assert joystick.get_axis(0) == 0.0, "Axis should be 0.0 after reset"
    assert joystick.get_hat(0) == (0, 0), "Hat should be centered after reset"
    print("   Reset test: PASS")

    # 测试状态对象
    print("\n6. Testing state object...")
    state = joystick.get_state()
    assert isinstance(state, JoystickState), "Should return JoystickState object"
    print(f"   State type: {type(state).__name__}")
    print("   State object: PASS")

    print("\n" + "=" * 50)
    print("All tests PASSED!")
    print("=" * 50)


def test_joystick_state():
    """测试手柄状态管理类"""
    print("\n" + "=" * 50)
    print("Testing JoystickState...")
    print("=" * 50)

    state = JoystickState()

    # 测试按键
    print("\n1. Testing button operations...")
    state.set_button(0, True)
    assert state.get_button(0) == True
    print("   Set button 0 to True: PASS")

    state.set_button(0, False)
    assert state.get_button(0) == False
    print("   Set button 0 to False: PASS")

    # 测试轴
    print("\n2. Testing axis operations...")
    state.set_axis(0, 0.5)
    assert abs(state.get_axis(0) - 0.5) < 0.01
    print("   Set axis 0 to 0.5: PASS")

    # 测试轴值限制
    state.set_axis(0, 2.0)
    assert state.get_axis(0) == 1.0
    print("   Axis value clamping: PASS")

    # 测试扳机轴
    state.set_axis(2, -0.5)
    assert state.get_axis(2) == 0.0
    print("   Trigger axis negative clamping: PASS")

    # 测试方向键
    print("\n3. Testing hat operations...")
    state.set_hat(0, (1, 1))
    assert state.get_hat(0) == (1, 1)
    print("   Set hat to (1, 1): PASS")

    # 测试重置
    print("\n4. Testing reset...")
    state.set_button(1, True)
    state.set_axis(1, -0.8)
    state.set_hat(0, (-1, 0))
    state.reset()
    assert state.get_button(1) == False
    assert state.get_axis(1) == 0.0
    assert state.get_hat(0) == (0, 0)
    print("   Reset all states: PASS")

    # 测试字符串表示
    print("\n5. Testing string representation...")
    state.set_button(0, True)
    state.set_axis(0, 0.5)
    state_str = str(state)
    assert "Button 0: PRESSED" in state_str
    assert "Axis 0: 0.500" in state_str
    print("   String representation: PASS")

    print("\n" + "=" * 50)
    print("All JoystickState tests PASSED!")
    print("=" * 50)


def test_compatibility():
    """测试与pygame的兼容性"""
    print("\n" + "=" * 50)
    print("Testing Pygame Compatibility...")
    print("=" * 50)

    joystick = VirtualJoystick(0)

    # 测试pygame风格的API
    print("\n1. Testing pygame-compatible API...")

    # 这些方法应该存在且可调用
    methods = [
        'get_init', 'get_id', 'get_instance_id', 'get_guid',
        'get_name', 'get_numbuttons', 'get_numaxes', 'get_numhats',
        'get_button', 'get_axis', 'get_hat', 'init', 'quit'
    ]

    for method in methods:
        assert hasattr(joystick, method), f"Missing method: {method}"
        print(f"   Method {method}: EXISTS")

    print("\n2. Testing method compatibility...")
    assert joystick.init() == True
    print("   init(): PASS")

    assert joystick.get_init() == True
    print("   get_init(): PASS")

    assert isinstance(joystick.get_name(), str)
    print("   get_name(): PASS")

    print("\n" + "=" * 50)
    print("Compatibility tests PASSED!")
    print("=" * 50)


if __name__ == "__main__":
    try:
        test_joystick_state()
        test_virtual_joystick()
        test_compatibility()
        print("\n" + "🎉 " * 10)
        print("ALL TESTS COMPLETED SUCCESSFULLY!")
        print("🎉 " * 10)
    except AssertionError as e:
        print(f"\n❌ TEST FAILED: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"\n❌ ERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
