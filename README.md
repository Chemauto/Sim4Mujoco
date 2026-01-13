# MuJoCo Car Control System / MuJoCo 小车控制系统

This directory contains a control system for the MuJoCo car simulation.
/ 本目录包含 MuJoCo 小车仿真的控制系统。

## Files / 文件说明

- **control.py**: Core car controller class that handles motor control / 核心小车控制器类，处理电机控制
- **keyboard.py**: Keyboard/mouse interface for manual car control / 键盘控制界面，用于手动控制小车
- **demo.py**: Demo scripts showing automatic control patterns / 演示脚本，展示自动控制模式
- **test.py**: Original test file (example) / 原始测试文件（示例）

## Car Model / 小车模型

The car has two actuators:
/ 小车有两个执行器：

- **forward motor**: Controls forward/backward movement (-1 to 1) / **前进电机**：控制前进/后退运动 (-1 到 1)
  - -1: Full reverse / 全速后退
  - 0: Stop / 停止
  - 1: Full forward / 全速前进

- **turn motor**: Controls turning (-1 to 1) / **转向电机**：控制转向 (-1 到 1)
  - -1: Full left turn / 全速左转
  - 0: Straight / 直行
  - 1: Full right turn / 全速右转

## Usage / 使用方法

### 1. Keyboard Control (Manual - ROS-style) / 键盘控制（手动 - ROS 风格）

Run the keyboard controller to manually drive the car:
/ 运行键盘控制器来手动驾驶小车：

```bash
python keyboard.py
```

**Controls (similar to ROS turtle teleop): / 控制键（类似 ROS 小海龟 teleop）：**
- `w` or `i`: Increase forward speed (press multiple times for more speed) / 增加前进速度（多次按键可加速）
- `s` or `k`: Decrease forward speed / move backward / 减少前进速度 / 后退
- `a` or `j`: Turn left / 左转
- `d` or `l`: Turn right / 右转
- `Space`: Stop immediately / 立即停止
- `q`: Quit simulation / 退出仿真

This is similar to the ROS `teleop_twist_keyboard` interface - each key press adjusts the speed incrementally.
/ 这类似于 ROS `teleop_twist_keyboard` 界面 - 每次按键都会增量调整速度。

### 2. Demo Scripts (Automatic) / 演示脚本（自动）

Run the demo to see automatic control patterns:
/ 运行演示以查看自动控制模式：

```bash
python demo.py
```

Available demos:
/ 可用演示：

1. **Circle**: Car drives in continuous circles / **圆形**：小车沿连续圆周运动
2. **Figure-8**: Car drives in a figure-8 pattern / **8 字形**：小车沿 8 字形路径运动
3. **Square**: Car drives in a square pattern / **方形**：小车沿方形路径运动

### 3. Using the Controller Class / 使用控制器类

You can also use the `CarController` class in your own scripts:
/ 你也可以在自己的脚本中使用 `CarController` 类：

```python
import mujoco
from control import CarController

# Load model / 加载模型
model = mujoco.MjModel.from_xml_path('../model/car.xml')
data = mujoco.MjData(model)

# Create controller / 创建控制器
controller = CarController(model, data)

# Set control signals / 设置控制信号
controller.set_control(forward=0.5, turn=0.3)

# Apply control (call before each mj_step) / 应用控制（在每次 mj_step 之前调用）
controller.apply_control()

# Step physics / 物理步进
mujoco.mj_step(model, data)

# Get car position / 获取小车位置
position = controller.get_car_position()
```

## Controller Methods / 控制器方法

### CarController Class

- `set_control(forward, turn)`: Set control signals (-1 to 1) / 设置控制信号 (-1 到 1)
- `apply_control()`: Apply control signals to actuators / 将控制信号应用到执行器
- `stop()`: Stop the car (set both controls to 0) / 停止小车（将两个控制设为 0）
- `get_forward_control()`: Get current forward control value / 获取当前前进控制值
- `get_turn_control()`: Get current turn control value / 获取当前转向控制值
- `get_car_position()`: Get current car position [x, y, z] / 获取小车当前位置 [x, y, z]
- `get_car_orientation()`: Get current car orientation as quaternion [w, x, y, z] / 获取小车当前姿态四元数 [w, x, y, z]

## Control System Architecture / 控制系统架构

The control system is organized as follows:
/ 控制系统组织如下：

1. **control.py**: Low-level motor control / 低层电机控制
   - Manages actuator IDs / 管理执行器 ID
   - Applies control signals to MuJoCo actuators / 将控制信号应用到 MuJoCo 执行器
   - Provides state query methods / 提供状态查询方法

2. **keyboard.py**: User interface / 用户界面
   - Handles keyboard input / 处理键盘输入
   - Implements smooth acceleration/turning / 实现平滑加速/转向
   - Calls control.py to apply signals / 调用 control.py 应用信号

3. **demo.py**: Example control patterns / 示例控制模式
   - Shows how to program automatic movements / 展示如何编程自动运动
   - Demonstrates state machines for complex patterns / 演示复杂模式的状态机

## Tips / 使用技巧

- Start with small control values (0.3-0.5) for smoother movements / 从较小的控制值（0.3-0.5）开始以获得更平滑的运动
- Use gradual changes to control values for realistic motion / 使用渐进式控制值变化以实现逼真的运动
- The car will continue moving until you set control to 0 or call `stop()` / 小车会持续运动，直到将控制设为 0 或调用 `stop()`
- Turn is more effective when moving forward / 前进时转向效果更好
- Control values are automatically clamped to [-1, 1] range / 控制值会自动限制在 [-1, 1] 范围内

## Requirements / 依赖要求

- MuJoCo (mujoco python package)
- NumPy

Install with:
/ 安装命令：

```bash
pip install mujoco numpy
```
