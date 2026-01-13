# MuJoCo Car Control System

This directory contains a control system for the MuJoCo car simulation.

## Files

- **control.py**: Core car controller class that handles motor control
- **keyboard.py**: Keyboard/mouse interface for manual car control
- **demo.py**: Demo scripts showing automatic control patterns
- **test.py**: Original test file (example)

## Car Model

The car has two actuators:
- **forward motor**: Controls forward/backward movement (-1 to 1)
  - -1: Full reverse
  - 0: Stop
  - 1: Full forward

- **turn motor**: Controls turning (-1 to 1)
  - -1: Full left turn
  - 0: Straight
  - 1: Full right turn

## Usage

### 1. Keyboard Control (Manual)

Run the keyboard controller to manually drive the car:

```bash
python keyboard.py
```

**Controls:**
- `Arrow Up` or `W`: Move forward
- `Arrow Down` or `S`: Move backward
- `Arrow Left` or `A`: Turn left
- `Arrow Right` or `D`: Turn right
- `Space`: Emergency stop

The car will accelerate gradually when you hold the keys and decelerate when released.

### 2. Demo Scripts (Automatic)

Run the demo to see automatic control patterns:

```bash
python demo.py
```

Available demos:
1. **Circle**: Car drives in continuous circles
2. **Figure-8**: Car drives in a figure-8 pattern
3. **Square**: Car drives in a square pattern

### 3. Using the Controller Class

You can also use the `CarController` class in your own scripts:

```python
import mujoco
from control import CarController

# Load model
model = mujoco.MjModel.from_xml_path('../model/car.xml')
data = mujoco.MjData(model)

# Create controller
controller = CarController(model, data)

# Set control signals
controller.set_control(forward=0.5, turn=0.3)

# Apply control (call before each mj_step)
controller.apply_control()

# Step physics
mujoco.mj_step(model, data)

# Get car position
position = controller.get_car_position()
```

## Controller Methods

### CarController Class

- `set_control(forward, turn)`: Set control signals (-1 to 1)
- `apply_control()`: Apply control signals to actuators
- `stop()`: Stop the car (set both controls to 0)
- `get_forward_control()`: Get current forward control value
- `get_turn_control()`: Get current turn control value
- `get_car_position()`: Get current car position [x, y, z]
- `get_car_orientation()`: Get current car orientation as quaternion [w, x, y, z]

## Control System Architecture

The control system is organized as follows:

1. **control.py**: Low-level motor control
   - Manages actuator IDs
   - Applies control signals to MuJoCo actuators
   - Provides state query methods

2. **keyboard.py**: User interface
   - Handles keyboard input
   - Implements smooth acceleration/turning
   - Calls control.py to apply signals

3. **demo.py**: Example control patterns
   - Shows how to program automatic movements
   - Demonstrates state machines for complex patterns

## Tips

- Start with small control values (0.3-0.5) for smoother movements
- Use gradual changes to control values for realistic motion
- The car will continue moving until you set control to 0 or call `stop()`
- Turn is more effective when moving forward
- Control values are automatically clamped to [-1, 1] range

## Requirements

- MuJoCo (mujoco python package)
- NumPy

Install with:
```bash
pip install mujoco numpy
```
