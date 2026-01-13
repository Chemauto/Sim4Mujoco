"""
Demo script showing how to use the car controller
This demonstrates automatic control patterns without keyboard input
"""

import time
import mujoco
import mujoco.viewer
from control import CarController


def run_circle_demo(model_path, duration=30):
    """
    Run a demo where the car drives in a circle

    Args:
        model_path: Path to car XML model
        duration: Duration in seconds
    """
    # Load model
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    # Create controller
    controller = CarController(model, data)

    print("\n" + "="*50)
    print("Circle Demo - Car will drive in circles")
    print("="*50 + "\n")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        start_time = time.time()

        while viewer.is_running() and (time.time() - start_time) < duration:
            step_start = time.time()

            # Control logic for circle: forward + constant turn
            controller.set_control(forward=0.5, turn=0.3)
            controller.apply_control()

            # Step physics
            mujoco.mj_step(model, data)

            # Print status
            if int(data.time * 100) % 50 == 0:
                pos = controller.get_car_position()
                print(f"\rTime: {data.time:.2f}s | Position: [{pos[0]:.2f}, {pos[1]:.2f}]", end="")

            # Sync viewer
            viewer.sync()

            # Time keeping
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)


def run_figure_eight_demo(model_path, duration=40):
    """
    Run a demo where the car drives in a figure-8 pattern

    Args:
        model_path: Path to car XML model
        duration: Duration in seconds
    """
    # Load model
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    # Create controller
    controller = CarController(model, data)

    print("\n" + "="*50)
    print("Figure-8 Demo - Car will drive in figure-8 pattern")
    print("="*50 + "\n")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        start_time = time.time()

        while viewer.is_running() and (time.time() - start_time) < duration:
            step_start = time.time()

            # Control logic for figure-8: varying turn direction
            # Use sine wave to alternate turning
            turn = 0.5 * (time.time() - start_time)
            controller.set_control(forward=0.6, turn=turn)
            controller.apply_control()

            # Step physics
            mujoco.mj_step(model, data)

            # Print status
            if int(data.time * 100) % 50 == 0:
                pos = controller.get_car_position()
                print(f"\rTime: {data.time:.2f}s | Position: [{pos[0]:.2f}, {pos[1]:.2f}]", end="")

            # Sync viewer
            viewer.sync()

            # Time keeping
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)


def run_square_demo(model_path, duration=30):
    """
    Run a demo where the car drives in a square

    Args:
        model_path: Path to car XML model
        duration: Duration in seconds
    """
    # Load model
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    # Create controller
    controller = CarController(model, data)

    print("\n" + "="*50)
    print("Square Demo - Car will drive in a square pattern")
    print("="*50 + "\n")

    # State machine for square pattern
    # States: 0=forward, 1=turn_left, 2=forward, 3=turn_left, etc.
    state = 0
    state_timer = 0
    forward_duration = 3.0  # seconds
    turn_duration = 1.0     # seconds

    with mujoco.viewer.launch_passive(model, data) as viewer:
        start_time = time.time()

        while viewer.is_running() and (time.time() - start_time) < duration:
            step_start = time.time()

            # State machine logic
            if state == 0:  # Moving forward
                controller.set_control(forward=0.7, turn=0.0)
                state_timer += model.opt.timestep
                if state_timer >= forward_duration:
                    state = 1
                    state_timer = 0
            elif state == 1:  # Turning
                controller.set_control(forward=0.3, turn=-0.8)
                state_timer += model.opt.timestep
                if state_timer >= turn_duration:
                    state = 0
                    state_timer = 0

            controller.apply_control()

            # Step physics
            mujoco.mj_step(model, data)

            # Print status
            if int(data.time * 100) % 50 == 0:
                pos = controller.get_car_position()
                print(f"\rTime: {data.time:.2f}s | State: {state} | Position: [{pos[0]:.2f}, {pos[1]:.2f}]", end="")

            # Sync viewer
            viewer.sync()

            # Time keeping
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)


def main():
    """Main entry point - run different demos"""
    model_path = '../model/car.xml'

    print("\n" + "="*50)
    print("Car Controller Demo")
    print("="*50)
    print("\nSelect demo:")
    print("  1 - Circle (car drives in circles)")
    print("  2 - Figure-8 (car drives in figure-8 pattern)")
    print("  3 - Square (car drives in a square)")

    try:
        choice = input("\nEnter choice (1-3): ").strip()

        if choice == '1':
            run_circle_demo(model_path)
        elif choice == '2':
            run_figure_eight_demo(model_path)
        elif choice == '3':
            run_square_demo(model_path)
        else:
            print("Invalid choice. Running circle demo...")
            run_circle_demo(model_path)

    except KeyboardInterrupt:
        print("\n\nDemo stopped by user")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
