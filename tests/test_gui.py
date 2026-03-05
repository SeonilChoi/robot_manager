"""GUI and tests: run GUI with `python tests/test_gui.py`, tests with pytest or `python tests/test_gui.py --test`."""

import sys
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
CONFIG_PATH = PROJECT_ROOT / "config" / "robot_config.yaml"


# -----------------------------------------------------------------------------
# Tests (pytest or: python tests/test_gui.py --test)
# -----------------------------------------------------------------------------

class TestGui(unittest.TestCase):
    def test_config_exists(self):
        self.assertTrue(CONFIG_PATH.exists(), f"Config not found: {CONFIG_PATH}")

    def test_robot_manager_initializes_with_config(self):
        from robot_manager import RobotManager
        manager = RobotManager(str(CONFIG_PATH))
        manager.initialize()
        self.assertIsNotNone(manager._robot)


# -----------------------------------------------------------------------------
# GUI (python tests/test_gui.py)
# -----------------------------------------------------------------------------

def main_gui() -> None:
    import tkinter as tk
    from tkinter import ttk, messagebox
    import numpy as np
    from robot_manager import JointState, RobotManager

    try:
        manager = RobotManager(str(CONFIG_PATH))
    except Exception as e:
        messagebox.showerror("Import Error", f"Could not create RobotManager: {e}")
        return

    if not CONFIG_PATH.exists():
        messagebox.showerror("Config Error", f"Config not found: {CONFIG_PATH}")
        return

    manager.initialize()

    root = tk.Tk()
    root.title("Robot Manager")
    root.resizable(False, False)

    main_frame = ttk.Frame(root, padding=20)
    main_frame.pack(fill=tk.BOTH, expand=True)

    status_var = tk.StringVar(value="Ready")

    def on_home() -> None:
        status_var.set("Homing...")
        root.update_idletasks()
        try:
            manager.home()
            status_var.set("Home done")
        except Exception as e:
            status_var.set("Error")
            messagebox.showerror("Home", str(e))

    def on_move() -> None:
        status_var.set("Moving...")
        root.update_idletasks()
        try:
            manager.move()
            status_var.set("Move done")
        except Exception as e:
            status_var.set("Error")
            messagebox.showerror("Move", str(e))

    def on_stop() -> None:
        status_var.set("Stopping...")
        root.update_idletasks()
        try:
            manager.stop()
            status_var.set("Stopped")
        except Exception as e:
            status_var.set("Error")
            messagebox.showerror("Stop", str(e))

    def on_auto() -> None:
        status_var.set("Auto running...")
        root.update_idletasks()
        try:
            manager.home()
            manager.move()
            status_var.set("Auto done")
        except Exception as e:
            status_var.set("Error")
            messagebox.showerror("Auto", str(e))

    def tick() -> None:
        try:
            manager.update(status)
        except Exception:
            pass
        root.after(10, tick)

    def control() -> None:
        try:
            control_state = manager.control()
            if control_state is not None:
                np.copyto(status.position, control_state.position)
                np.copyto(status.velocity, control_state.velocity)
                np.copyto(status.torque, control_state.torque)
        except Exception:
            pass
        root.after(100, control)

    ttk.Label(main_frame, text="Robot Manager", font=("", 14, "bold")).pack(pady=(0, 16))

    btn_frame = ttk.Frame(main_frame)
    btn_frame.pack(fill=tk.X, pady=8)

    ttk.Button(btn_frame, text="Home", command=on_home).pack(side=tk.LEFT, padx=4)
    ttk.Button(btn_frame, text="Move", command=on_move).pack(side=tk.LEFT, padx=4)
    ttk.Button(btn_frame, text="Stop", command=on_stop).pack(side=tk.LEFT, padx=4)
    ttk.Button(btn_frame, text="Auto", command=on_auto).pack(side=tk.LEFT, padx=4)

    ttk.Separator(main_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=12)
    ttk.Label(main_frame, textvariable=status_var).pack(anchor=tk.W)

    status = JointState(
        id=np.arange(manager._robot._number_of_joints),
        position=np.zeros(manager._robot._number_of_joints),
        velocity=np.zeros(manager._robot._number_of_joints),
        torque=np.zeros(manager._robot._number_of_joints),
    )

    root.after(10, tick)
    root.after(100, control)
    root.mainloop()


if __name__ == "__main__":
    if "--test" in sys.argv:
        sys.argv = [sys.argv[0]] + [a for a in sys.argv[1:] if a != "--test"]
        unittest.main()
    else:
        main_gui()
