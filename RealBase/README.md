# 3全向轮底盘控制模块

## 概述

这个模块提供了控制3全向轮底盘的完整接口。底盘使用3个Feetech STS3215舵机作为驱动电机。

**重要**: 本模块已完全独立,包含了所有必需的motors驱动代码,不再依赖TestSo101文件夹。

## 目录结构

```
TestBase/
├── motor_controller.py      # 底盘控制器主模块
├── test_kinematics.py       # 运动学测试程序 (推荐!)
├── example_usage.py         # 使用示例代码
├── test_imports.py          # 导入测试脚本
├── requirements.txt         # Python依赖包列表
├── README.md                # 本文档
└── motors/                  # 电机驱动模块(已包含)
    ├── __init__.py
    ├── motors_bus.py        # 电机总线抽象层
    ├── calibration_gui.py   # 校准工具
    ├── feetech/             # Feetech舵机驱动
    │   ├── __init__.py
    │   ├── feetech.py       # Feetech电机总线实现
    │   └── tables.py        # 控制表和参数
    └── utils/               # 工具函数
        ├── __init__.py
        ├── encoding_utils.py
        └── utils.py
```

## 快速开始

### 1. 安装依赖

```bash
cd /home/dora/RoboOs/BaseControl/TestBase
pip install -r requirements.txt
```

### 2. 测试导入

```bash
python3 test_imports.py
```

如果看到 "所有模块导入成功!" 说明一切正常。

### 3. 最简单的使用示例

```python
from motor_controller import OmniWheelController

# 创建底盘控制器
base = OmniWheelController(port="/dev/ttyACM0")

# 连接
if base.connect():
    # 方法1: 使用运动学控制(推荐)
    base.set_velocity(linear_speed=0.2, vx=1, vy=0, omega=0)  # 向前

    import time
    time.sleep(2)

    # 停止
    base.stop()
    base.disconnect()
```

### 4. 运行运动学测试 (推荐!)

```bash
python3 test_kinematics.py
```

这个程序提供交互式菜单,可以测试:
- 前进/后退
- 左移/右移
- 斜向运动
- 原地旋转
- 组合运动

### 5. 运行完整示例

```bash
python3 example_usage.py
```

## 主要功能

### 1. 单轮控制

控制单个轮子的转动方向和速度。

**函数:**
```python
control_wheel(wheel_id, direction, speed)
```

**参数:**
- `wheel_id`: 电机ID (使用 `base.WHEEL_IDS[0/1/2]` 获取)
- `direction`: 转动方向 (1=正转, -1=反转)
- `speed`: 转动速度 (0-100, 0表示停止)

**示例:**
```python
from motor_controller import OmniWheelController

base = OmniWheelController(port="/dev/ttyACM0")
base.connect()

# 轮子1正转,速度50%
base.control_wheel(wheel_id=base.WHEEL_IDS[0], direction=1, speed=50)

# 轮子2反转,速度30%
base.control_wheel(wheel_id=base.WHEEL_IDS[1], direction=-1, speed=30)

# 停止轮子1
base.stop_wheel(base.WHEEL_IDS[0])

# 停止所有轮子
base.stop()
```

### 2. 运动学控制(推荐)

使用速度向量控制底盘整体运动,包括前进、横移和旋转。

**函数:**
```python
set_velocity(linear_speed, vx, vy, omega)
set_velocity_raw(vx, vy, omega)
```

**参数:**
- `linear_speed`: 速度大小 (m/s)
- `vx`: x方向分量 (归一化, -1到1)
- `vy`: y方向分量 (归一化, -1到1)
- `omega`: 旋转角速度 (rad/s, 正值逆时针)

**示例:**
```python
# 向前 0.2 m/s
base.set_velocity(linear_speed=0.2, vx=1, vy=0, omega=0)

# 向左横移 0.2 m/s
base.set_velocity(linear_speed=0.2, vx=0, vy=1, omega=0)

# 原地旋转 0.5 rad/s
base.set_velocity(linear_speed=0, vx=0, vy=0, omega=0.5)

# 前进同时旋转
base.set_velocity(linear_speed=0.2, vx=1, vy=0, omega=0.5)

# 直接设置速度(不归一化)
base.set_velocity_raw(vx=0.1, vy=0.1, omega=0.5)
```

### 3. 状态查询

查询轮子当前速度状态。

**函数:**
```python
# 获取轮子角速度 (rad/s)
get_wheel_velocities()

# 获取单个轮子速度信息
get_wheel_speed(wheel_id)

# 获取所有轮子速度信息
get_all_speeds()
```

**示例:**
```python
# 查询轮子角速度 (rad/s)
wheel_vels = base.get_wheel_velocities()
print(f"轮子角速度: {wheel_vels} rad/s")

# 查询轮子1的速度
result = base.get_wheel_speed(base.WHEEL_IDS[0])
print(f"速度: {result['speed']}%, 方向: {result['direction_str']}")

# 查询所有轮子速度
result = base.get_all_speeds()
for wheel, info in result['speeds'].items():
    print(f"{wheel}: {info['direction_str']}, 速度={info['speed']}%")
```

## 硬件配置

### 电机布置

3全向轮底盘的轮子布置:
- 轮子1 (wheel_1): 前方 (0度方向)
- 轮子2 (wheel_2): 右后方 (120度方向)
- 轮子3 (wheel_3): 左后方 (240度方向)

### 电机参数

- 型号: Feetech STS3215
- 通信协议: SCServo
- 控制模式: 速度模式

### 机械参数

在初始化时可以设置:
- `wheel_radius`: 轮子半径(米),默认0.05m(5cm)
- `robot_radius`: 机器人半径(米),默认0.15m(从中心到轮子的距离)

## 电机ID配置

### 当前配置

底盘控制器的电机ID默认配置在 `base_controller.py` 第22行:

```python
class OmniWheelController:
    # 电机ID配置 - 修改这里即可更改所有电机ID
    WHEEL_IDS = [13, 14, 15]  # 当前配置
```

### 如何修改电机ID

**方法1: 修改源码(推荐)**

编辑 `base_controller.py` 第22行:

```python
WHEEL_IDS = [1, 2, 3]  # 改为你的电机ID
```

**方法2: 运行时修改**

在使用前修改类变量:

```python
from base_controller import OmniWheelController

# 修改电机ID
OmniWheelController.WHEEL_IDS = [1, 2, 3]

# 创建控制器
base = OmniWheelController(port="/dev/ttyACM0")
base.connect()
```

### 电机ID映射关系

```
WHEEL_IDS[0] → 轮子1 (wheel_1, 前方)
WHEEL_IDS[1] → 轮子2 (wheel_2, 右后方)
WHEEL_IDS[2] → 轮子3 (wheel_3, 左后方)
```

### 注意事项

1. **ID必须唯一**: 确保3个电机的ID各不相同
2. **ID范围**: 通常在1-254之间
3. **修改后生效**: 修改源码后需要重新运行程序
4. **查看当前配置**:
   ```python
   from base_controller import OmniWheelController
   print("当前电机ID配置:", OmniWheelController.WHEEL_IDS)
   ```

## 依赖项

### 主要依赖包

- numpy - 数值计算
- pyserial - 串口通信
- scservo-sdk - Feetech舵机SDK
- draccus - 配置文件解析
- deepdiff - 数据对比
- tqdm - 进度条

### 安装方法

```bash
pip install -r requirements.txt
```

## 注意事项

1. **端口配置**: 确保串口端口正确
   ```bash
   # 查看可用串口
   ls /dev/tty*
   ```
   常见串口名称: `/dev/ttyACM0`, `/dev/ttyACM1`, `/dev/ttyUSB0`

2. **电机ID**: 确保电机ID配置正确(参见上面的电机ID配置章节)

3. **速度限制**: 速度参数范围为0-100,建议从低速开始测试

4. **安全**: 使用前确保轮子悬空或放在可自由移动的平面上

5. **异常处理**: 建议使用try-except捕获异常,确保程序退出前停止所有电机

## 故障排除

### 连接失败
- 检查串口端口是否正确
- 检查USB线缆是否连接
- 检查电机电源是否开启

### 电机不响应
- 检查电机ID是否正确
- 检查电机是否已使能扭矩
- 尝试重新连接

### 运动不正常
- 检查轮子方向定义是否符合实际
- 调整`robot_radius`参数
- 检查轮子半径参数`wheel_radius`

### 导入失败
确保已安装所有依赖:
```bash
pip install -r requirements.txt
```

## API参考

### OmniWheelController类

#### 初始化
```python
OmniWheelController(port, wheel_radius, robot_radius)
```
- `port`: 串口端口
- `wheel_radius`: 轮子半径(米),默认0.05
- `robot_radius`: 机器人半径(米),默认0.15

#### 主要方法

**connect()** - 连接底盘电机
```python
success = base.connect()
```

**disconnect()** - 断开连接
```python
base.disconnect()
```

**control_wheel(wheel_id, direction, speed)** - 控制单个轮子
```python
result = base.control_wheel(wheel_id=base.WHEEL_IDS[0], direction=1, speed=50)
```

**stop_wheel(wheel_id)** - 停止指定轮子
```python
result = base.stop_wheel(base.WHEEL_IDS[0])
```

**stop()** - 停止所有轮子
```python
success = base.stop()
```

**set_velocity(linear_speed, vx, vy, omega)** - 运动学控制(推荐)
```python
success = base.set_velocity(linear_speed=0.2, vx=1, vy=0, omega=0)
```

**set_velocity_raw(vx, vy, omega)** - 直接速度控制
```python
success = base.set_velocity_raw(vx=0.1, vy=0.1, omega=0.5)
```

**get_wheel_velocities()** - 获取轮子角速度
```python
wheel_vels = base.get_wheel_velocities()  # numpy数组,单位rad/s
```

**control_wheel(wheel_id, direction, speed)** - 控制单个轮子
```python
result = base.control_wheel(wheel_id=base.WHEEL_IDS[0], direction=1, speed=50)
```

**stop_wheel(wheel_id)** - 停止指定轮子
```python
result = base.stop_wheel(base.WHEEL_IDS[0])
```

**get_wheel_speed(wheel_id)** - 获取轮子速度
```python
result = base.get_wheel_speed(base.WHEEL_IDS[0])
```

**get_all_speeds()** - 获取所有轮子速度
```python
result = base.get_all_speeds()
```

完整的API文档详见`motor_controller.py`中的函数文档字符串。

## 需要帮助?

如果遇到问题:
1. 查看"故障排除"章节
2. 运行 `python3 test_imports.py` 检查环境
3. 运行 `python3 example_usage.py` 查看示例
4. 检查 `base_controller.py` 中的函数文档字符串
