# Xbox手柄模拟器 - 快速使用指南

## 启动程序

在项目目录下运行：
```bash
./run.sh
```

或者直接运行：
```bash
python src/main.py
```

## 界面说明

### 手柄布局
```
┌─────────────────────────────────────────┐
│  [LB]                    [RB]           │  ← 肩键
│                                         │
│      [LT]              [RT]             │  ← 扳机
│                                         │
│                                         │
│         [Y]              [D-PAD]        │
│
│      [X] [A] [B]                        │  ← ABXY按键
│                                         │
│                                         │
│    [左摇杆]          [右摇杆]            │  ← 双轴摇杆
│       (  )            (  )             │
│                                         │
│           [SEL] [STA]                  │  ← 功能键
└─────────────────────────────────────────┘
```

### 按键说明

**按钮（点击使用）**
- **A (绿色)** - button_id = 0
- **B (红色)** - button_id = 1
- **X (蓝色)** - button_id = 2
- **Y (黄色)** - button_id = 3
- **LB** - 左肩键，button_id = 4
- **RB** - 右肩键，button_id = 5
- **SEL** - 选择键，button_id = 6
- **STA** - 开始键，button_id = 7

**摇杆（拖拽使用）**
- **左摇杆** - 控制轴0(X)和轴1(Y)，值域 [-1, 1]
- **右摇杆** - 控制轴3(X)和轴4(Y)，值域 [-1, 1]
  - 鼠标按下并拖拽可移动摇杆
  - 释放后自动回弹到中心

**扳机（点击使用）**
- **LT** - 左扳机，axis_id = 2，值域 [0, 1]
- **RT** - 右扳机，axis_id = 5，值域 [0, 1]
  - 点击位置不同会产生不同的压力值

**方向键（D-PAD）**
- 支持8个方向：上、下、左、右及对角线
- 返回值：(x, y) 元组
  - x: -1(左), 0(中), 1(右)
  - y: -1(下), 0(中), 1(上)

### 状态显示

底部面板实时显示所有按键和轴的当前状态，包括：
- 按下状态（PRESSED）
- 轴值（精确到小数点后3位）
- 方向键状态

## 编程接口示例

### 基本使用
```python
from src.virtual_joystick import VirtualJoystick

# 创建虚拟手柄
joystick = VirtualJoystick(0)

# 读取按键状态
if joystick.get_button(0):  # A键
    print("A键按下")

# 读取摇杆位置
left_x = joystick.get_axis(0)  # 左摇杆X轴 [-1, 1]
left_y = joystick.get_axis(1)  # 左摇杆Y轴 [-1, 1]

# 读取扳机值
left_trigger = joystick.get_axis(2)  # LT [0, 1]
right_trigger = joystick.get_axis(5)  # RT [0, 1]

# 读取方向键
hat_x, hat_y = joystick.get_hat(0)
```

### 与robot control系统集成
```python
# 替换原来的pygame手柄
import sys
sys.path.insert(0, '/path/to/Project4_SimJoy')

from src.virtual_joystick import VirtualJoystick

# 创建虚拟手柄（与pygame.joystick.Joystick接口兼容）
joystick = VirtualJoystick(0)
joystick.init()

# 后续代码无需修改，直接使用
# 例如：读取按键控制机器人运动
if joystick.get_button(0):  # A键
    # 执行某个动作
    pass

# 读取摇杆控制机器人移动
lx = joystick.get_axis(0)
ly = joystick.get_axis(1)
```

## 常见问题

**Q: 程序无法启动？**
A: 确保已安装PyQt5：
```bash
pip install -r requirements.txt
```

**Q: 如何测试功能是否正常？**
A: 运行单元测试：
```bash
python test/test_joystick.py
```

**Q: 如何自定义手柄外观？**
A: 修改 `src/components/` 目录下的组件文件，调整颜色和尺寸参数。

**Q: 可以同时使用多个虚拟手柄吗？**
A: 可以，创建多个VirtualJoystick实例即可，只需指定不同的ID：
```python
joy1 = VirtualJoystick(0)
joy2 = VirtualJoystick(1)
```

## 技术支持

如有问题或建议，请查看README.md或提交Issue。
