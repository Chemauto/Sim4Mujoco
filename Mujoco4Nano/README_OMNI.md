# 三麦克纳姆轮底盘控制系统完整指南

## 📋 概述

本目录包含三轮全向移动平台(3WD Omni)的完整控制系统,包括:

- **底层运动控制**: `OmniWheelController` - 基于LeKiwi官方运动学实现
- **全局导航控制**: `GlobalNavigator` - 基于PID的点到点位置控制

**运动学来源**: LeKiwi官方实现

## 📁 文件说明

| 文件 | 说明 |
|------|------|
| `omni_controller.py` | 底层运动控制器 (速度控制) |
| `global_navigator.py` | 全局导航控制器 (位置控制) ⭐ |
| `test_omni_viewer.py` | 底层控制器可视化演示 |
| `test_global_navigation.py` | 全局导航测试程序 ⭐ |
| `README_OMNI.md` | 本文档 |

---

## 🎯 两层控制架构

### 1. 底层运动控制 (OmniWheelController)
**用途**: 直接控制机器人速度
- 输入: 速度指令 (vx, vy, omega)
- 输出: 轮子角速度
- 适用场景: 遥控、手动控制、简单轨迹

### 2. 全局导航控制 (GlobalNavigator)
**用途**: 点到点位置控制
- 输入: 目标位置 (x, y, yaw)
- 输出: 机器人速度 (通过PID自动计算)
- 适用场景: 自主导航、精确定位、路径规划

```
目标位置 → GlobalNavigator (PID) → 速度指令 → OmniWheelController → 轮子控制
```

---

## 🚀 快速开始

### 方式1: 底层速度控制演示

```bash
cd /home/dora/RoboOs/New/Sim/Sim4Mujoco/Mujoco4Nano
python test_omni_viewer.py
```

选择演示模式:
1. 自动演示 (9种运动模式)
2. 圆形轨迹
3. 方形轨迹
4. 简单演示
5. 手动控制

### 方式2: 全局导航演示 ⭐

```bash
python test_global_navigation.py
```

选择演示模式:
1. **单点移动** - 移动到一个目标点
2. **多点移动** - 方形路径
3. **带姿态控制** - 位置+姿态同时控制
4. **手动输入** - 自定义坐标和角度

---

## 💻 在代码中使用

### 场景1: 底层速度控制 (手动/遥控)

```python
import mujoco
import mujoco.viewer
from omni_controller import OmniWheelController

# 加载模型
model = mujoco.MjModel.from_xml_path('model/assets/scene.xml')
data = mujoco.MjData(model)
controller = OmniWheelController(model, data)

# 速度控制
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        # 前进 0.5 m/s
        controller.set_velocity(linear_speed=0.5, vx=1, vy=0, omega=0)
        controller.apply_control()
        mujoco.mj_step(model, data)
        viewer.sync()
```

### 场景2: 全局导航 (自动定位) ⭐

```python
import mujoco
import mujoco.viewer
from omni_controller import OmniWheelController
from global_navigator import GlobalNavigator

# 加载模型
model = mujoco.MjModel.from_xml_path('model/assets/scene.xml')
data = mujoco.MjData(model)

# 创建控制器
omni_controller = OmniWheelController(model, data)
navigator = GlobalNavigator(model, data)

# 设置目标: 移动到 (0.5, 0.3), 朝向90度
navigator.set_target(x=0.5, y=0.3, z=0, yaw=np.pi/2)

# 导航循环
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        # PID自动计算速度
        vx, vy, omega = navigator.update(model.opt.timestep)

        # 应用到底层控制器
        omni_controller.set_velocity_raw(vx, vy, omega)
        omni_controller.apply_control()

        # 仿真步进
        mujoco.mj_step(model, data)
        viewer.sync()

        # 检查是否到达
        if not navigator.is_navigating:
            print("到达目标!")
            break
```

### 场景3: 多点路径规划

```python
# 定义路径点
waypoints = [
    (0.5, 0.0, None),      # 右边, 不控制姿态
    (0.5, 0.5, np.pi/2),  # 右上, 朝向90度
    (0.0, 0.5, np.pi),    # 上边, 朝向180度
    (0.0, 0.0, None),     # 回到起点
]

current = 0

with viewer:
    while viewer.is_running():
        # 设置下一个目标
        if not navigator.is_navigating and current < len(waypoints):
            x, y, yaw = waypoints[current]
            navigator.set_target(x, y, 0, yaw)
            current += 1

        # 导航控制...
        vx, vy, omega = navigator.update(model.opt.timestep)
        omni_controller.set_velocity_raw(vx, vy, omega)
        omni_controller.apply_control()
        mujoco.mj_step(model, data)
        viewer.sync()
```

---

## 📚 API参考

### OmniWheelController (底层控制器)

#### 主要方法

| 方法 | 说明 |
|------|------|
| `set_velocity(linear_speed, vx, vy, omega=0)` | 设置速度 (推荐) |
| `set_velocity_raw(vx, vy, omega=0)` | 直接设置速度分量 |
| `apply_control()` | 应用控制到执行器 |
| `stop()` | 停止所有轮子 |
| `get_robot_position()` | 返回 [x, y, z] 位置 |
| `get_robot_orientation()` | 返回 [w, x, y, z] 四元数 |

#### set_velocity() 参数

| 参数 | 类型 | 范围 | 说明 |
|------|------|------|------|
| `linear_speed` | float | ≥0 | 速度大小 (m/s) |
| `vx` | float | -1到1 | x方向分量, 1=向前 |
| `vy` | float | -1到1 | y方向分量, 1=向左 |
| `omega` | float | 任意 | 旋转角速度 (rad/s) |

**示例:**
```python
# 前进 0.5 m/s
controller.set_velocity(0.5, 1, 0, 0)

# 左移 0.3 m/s
controller.set_velocity(0.3, 0, 1, 0)

# 原地逆时针旋转
controller.set_velocity(0, 0, 0, 1)

# 前进同时旋转
controller.set_velocity(0.3, 1, 0, 0.5)
```

---

### GlobalNavigator (全局导航控制器) ⭐

#### 主要方法

| 方法 | 说明 |
|------|------|
| `set_target(x, y, z=0, yaw=None)` | 设置目标位置和姿态 |
| `update(dt)` | 更新导航并返回速度 |
| `get_navigation_status()` | 获取导航状态 |
| `is_target_reached()` | 检查是否到达 |
| `stop_navigation()` | 停止导航 |

#### set_target() 参数

| 参数 | 类型 | 说明 |
|------|------|------|
| `x` | float | 目标x坐标 (米) |
| `y` | float | 目标y坐标 (米) |
| `z` | float | 目标z坐标 (米), 默认0 |
| `yaw` | float/None | 目标朝向 (弧度), None=不控制姿态 |

**角度说明:**
- `0` → 指向+x (机器人前方)
- `π/2` → 指向+y (机器人左侧)
- `-π/2` → 指向-y (机器人右侧)
- `π` → 指向-x (机器人后方)

**示例:**
```python
# 只控制位置,不控制姿态
navigator.set_target(x=0.5, y=0.3, yaw=None)

# 同时控制位置和姿态
navigator.set_target(x=0.5, y=0.3, yaw=np.pi/2)
```

#### get_navigation_status() 返回字段

```python
status = navigator.get_navigation_status()
# {
#     'current_position': [x, y, z],
#     'current_yaw': angle,
#     'is_navigating': True/False,
#     'target_position': [x, y, z],  # 如果正在导航
#     'position_error': error_meters,
#     'yaw_error': error_radians     # 如果控制姿态
# }
```

---

## 🎯 坐标系说明

### 世界坐标系
- MuJoCo的全局坐标系,固定不动
- 原点在机器人初始位置
- **x轴**: 机器人初始前方
- **y轴**: 机器人初始左侧
- **z轴**: 垂直向上

### 机器人坐标系
- 原点在机器人底盘中心,随机器人移动
- **x轴**: 机器人当前正前方
- **y轴**: 机器人当前左侧
- **z轴**: 垂直向上

### 坐标转换

**底层控制** - 使用机器人坐标系:
```python
controller.set_velocity(linear_speed=0.5, vx=1, vy=0, omega=0)
# vx, vy 是机器人坐标系
```

**全局导航** - 使用世界坐标系:
```python
navigator.set_target(x=0.5, y=0.3, yaw=np.pi/2)
# x, y 是世界坐标系
# 控制器自动转换到机器人坐标系
```

---

## ⚙️ 系统参数

### 硬件参数

| 参数 | 值 |
|------|-----|
| 执行器速度范围 | -3.14 到 3.14 rad/s |
| 轮子半径 | 0.051 m |
| 机器人半径 | 0.0923 m |

### 推荐参数

| 类型 | 速度/角速度 |
|------|------------|
| 精细操作 | 0.2-0.3 m/s |
| 一般移动 | 0.4-0.5 m/s |
| 快速移动 | 0.5-0.8 m/s |
| 精细旋转 | 0.3-0.5 rad/s |
| 一般旋转 | 0.8-1.0 rad/s |
| 快速旋转 | 1.5-2.0 rad/s |

### PID参数 (GlobalNavigator)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `kp_x` | 1.0 | 位置比例增益 |
| `ki_x` | 0.01 | 位置积分增益 |
| `kd_x` | 0.1 | 位置微分增益 |
| `kp_yaw` | 2.0 | 姿态比例增益 |
| `ki_yaw` | 0.01 | 姿态积分增益 |
| `kd_yaw` | 0.2 | 姿态微分增益 |
| 位置容差 | 0.02 m | 2cm |
| 角度容差 | 0.05 rad | ~3度 |

---

## 🔧 运动学实现

### 轮子配置
- **布局**: 120°间隔的三轮全向轮
- **左轮**: 30° to forward direction
- **右轮**: 150° from forward
- **后轮**: 270° from forward

### 运动学矩阵

使用**LeKiwi官方**的标准运动学矩阵:

```python
F_matrix = r * [[√3/2, -√3/2, 0],
                 [-1/2,  -1/2,   1],
                 [-1/(3L), -1/(3L), -1/(3L)]]
```

其中:
- `r` = 轮子半径 (0.051 m)
- `L` = 机器人半径 (0.0923 m)

这个矩阵将机器人速度 `[vx, vy, omega]` 映射到轮子角速度 `[left, right, back]`。

---

## 📖 代码示例

### 示例1: 简单前进 (底层控制)

```python
with mujoco.viewer.launch_passive(model, data) as viewer:
    start = time.time()
    while time.time() - start < 5:
        controller.set_velocity(0.5, 1, 0, 0)
        controller.apply_control()
        mujoco.mj_step(model, data)
        viewer.sync()
```

### 示例2: 方形路径 (底层控制)

```python
state = 0  # 0=前进, 1=旋转
timer = 0

with viewer:
    while viewer.is_running():
        if state == 0:  # 前进3秒
            controller.set_velocity(0.5, 1, 0, 0)
            timer += model.opt.timestep
            if timer >= 3.0:
                state = 1
                timer = 0
        elif state == 1:  # 旋转90度
            controller.set_velocity(0, 0, 0, 1)
            timer += model.opt.timestep
            if timer >= 1.57:
                state = 0
                timer = 0

        controller.apply_control()
        mujoco.mj_step(model, data)
        viewer.sync()
```

### 示例3: 精确定位 (全局导航) ⭐

```python
navigator.set_target(0.5, 0.3)

while navigator.is_navigating:
    vx, vy, omega = navigator.update(model.opt.timestep)
    controller.set_velocity_raw(vx, vy, omega)
    controller.apply_control()
    mujoco.mj_step(model, data)
    viewer.sync()
```

### 示例4: 实时状态显示

```python
navigator.set_target(0.5, 0.3)

while navigator.is_navigating:
    vx, vy, omega = navigator.update(model.opt.timestep)
    controller.set_velocity_raw(vx, vy, omega)
    controller.apply_control()
    mujoco.mj_step(model, data)
    viewer.sync()

    # 每秒打印一次状态
    if int(time.time() * 10) % 10 == 0:
        status = navigator.get_navigation_status()
        print(f"位置误差: {status['position_error']*100:.2f}cm")
```

---

## ❓ 常见问题

### 底层控制相关

#### Q: 有轻微偏转正常吗?
**A**: 是的,这是正常现象。可能原因:
- 物理仿真中的摩擦力不均匀
- 轮子制造公差
- 数值积分误差

偏转量通常很小(几厘米),实际应用中可接受。

#### Q: 如何精确旋转90度?
**A**: 使用时间控制:
```python
start = time.time()
while time.time() - start < 1.57:  # π/2 / 1.0
    controller.set_velocity(0, 0, 0, 1.0)
    controller.apply_control()
    mujoco.mj_step(model, data)
```

### 全局导航相关 ⭐

#### Q: 机器人没有到达目标点?
**A**: 可能原因:
1. 目标太远 - 超出运动范围
2. PID参数不合适 - 需要调整增益
3. 速度限制太低 - 修改 `output_limits`

**解决方法**:
```python
# 在 global_navigator.py 中调整
self.pid_x = PIDController(kp=1.0, ki=0.01, kd=0.1,
                           output_limits=(-1.5, 1.5))
```

#### Q: 运动过程中振荡严重?
**A**:
1. 降低比例增益 `kp`
2. 增加微分增益 `kd`
3. 减小速度限制

#### Q: 到达精度不够?
**A**:
1. 减小容差阈值:
```python
navigator.position_tolerance = 0.01  # 1cm
navigator.angle_tolerance = 0.02     # ~1度
```

2. 增加积分增益 `ki` 消除稳态误差

#### Q: 如何实现平滑路径规划?
**A**: 使用路径插值:
```python
import numpy as np

def interpolate_path(start, end, num_steps):
    path = []
    for i in range(num_steps + 1):
        t = i / num_steps
        point = start + t * (end - start)
        path.append(point)
    return path

# 使用
start = np.array([0.0, 0.0])
end = np.array([1.0, 1.0])
waypoints = interpolate_path(start, end, num_steps=10)

for point in waypoints:
    navigator.set_target(point[0], point[1])
    while navigator.is_navigating:
        # 仿真循环...
```

---

## 📖 相关资源

- **LeKiwi项目**: LeKiwi官方实现
- **MuJoCo文档**: https://mujoco.readthedocs.io/
- **scipy.spatial.transform**: 四元数和旋转矩阵转换

---

## 🎯 使用建议

### 底层控制 vs 全局导航

**使用底层控制** (OmniWheelController) 当:
- 需要实时遥控
- 简单的轨迹运动
- 已知速度指令

**使用全局导航** (GlobalNavigator) 当:
- 需要精确位置控制
- 点到点导航
- 需要姿态控制
- 自动驾驶/路径规划

### 组合使用

```python
# 方式1: 先导航,再手动微调
navigator.set_target(0.5, 0.3)
while navigator.is_navigating:
    # 自动导航...
    pass

# 到达后,手动调整
controller.set_velocity(0.1, 1, 0, 0)

# 方式2: 根据情况切换
if need_precise_position:
    vx, vy, omega = navigator.update(dt)
else:
    # 手动控制
    vx, vy, omega = manual_control_input
```

---

**版本**: 3.0 (集成全局导航)
**最后更新**: 2026-01-14
**状态**: ✅ 已测试验证
