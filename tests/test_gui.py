"""
GUI and tests for Robot Manager.

Run GUI: python tests/test_gui.py
Run tests: pytest tests/test_gui.py -v or python tests/test_gui.py --test
"""
import sys
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
CONFIG_PATH = PROJECT_ROOT / "config" / "robot_config.yaml"


# -----------------------------------------------------------------------------
# Tests (pytest or: python tests/test_gui.py --test)
# -----------------------------------------------------------------------------

class TestGui(unittest.TestCase):
    """Tests for config existence and RobotManager initialization."""

    def test_config_exists(self):
        self.assertTrue(CONFIG_PATH.exists(), f"Config not found: {CONFIG_PATH}")

    def test_robot_manager_initializes_with_config(self):
        from robot_manager import RobotManager
        manager = RobotManager(str(CONFIG_PATH))
        self.assertIsNotNone(manager._robot)


# -----------------------------------------------------------------------------
# GUI (python tests/test_gui.py)
# -----------------------------------------------------------------------------

def main_gui() -> None:
    """Launch the Robot Manager GUI (tkinter + matplotlib)."""
    import tkinter as tk
    from tkinter import ttk, messagebox
    import numpy as np
    from collections import deque
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    from matplotlib.figure import Figure
    from mpl_toolkits.mplot3d import Axes3D
    from robot_manager import RobotManager, JointState, SphereObstacleState, CircleObstacleState

    try:
        manager = RobotManager(str(CONFIG_PATH))
    except Exception as e:
        messagebox.showerror("Import Error", f"Could not create RobotManager: {e}")
        return

    if not CONFIG_PATH.exists():
        messagebox.showerror("Config Error", f"Config not found: {CONFIG_PATH}")
        return

    root = tk.Tk()
    root.title("Robot Manager")
    root.resizable(True, True)

    main_frame = ttk.Frame(root, padding=20)
    main_frame.pack(fill=tk.BOTH, expand=False)

    status_var = tk.StringVar(value="Ready")

    # Figure: 1 row, 2 columns; total size 2x (14, 12)
    fig = Figure(figsize=(14, 12))
    ax = fig.add_subplot(1, 1, 1, projection="3d")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")

    canvas = FigureCanvasTkAgg(fig, master=root)
    canvas.get_tk_widget().config(width=3000, height=2000)
    canvas.get_tk_widget().pack(padx=20, pady=(0, 20))

    # Link connectivity for 9-point two-chain layout (e.g. LittleReader: base 0, then chains 2-3-4 and 6-7-8)
    LINK_PAIRS = [(0, 1), (1, 2), (2, 3), (3, 4), (1, 5), (5, 6), (6, 7)]

    def _draw_sphere(ax, center: np.ndarray, radius: float, color: str = "gray", alpha: float = 0.4) -> None:
        u = np.linspace(0, 2 * np.pi, 16)
        v = np.linspace(0, np.pi, 12)
        x = center[0] + radius * np.outer(np.cos(u), np.sin(v))
        y = center[1] + radius * np.outer(np.sin(u), np.sin(v))
        z = center[2] + radius * np.outer(np.ones_like(u), np.cos(v))
        ax.plot_surface(x, y, z, color=color, alpha=alpha)

    def _draw_circle_obstacle(ax, center: np.ndarray, radius: float, color: str = "orange", alpha: float = 0.2) -> None:
        """Draw a 2D circle obstacle in the xy-plane at z=center[2] (surface-avoidance disk)."""
        theta = np.linspace(0, 2 * np.pi, 48)
        cx, cy, cz = center[0], center[1], center[2]
        x = cx + radius * np.cos(theta)
        y = cy + radius * np.sin(theta)
        z = np.full_like(theta, cz)
        ax.plot3D(x, y, z, color=color, linewidth=2)
        # Filled disk in the plane
        r_ = np.linspace(0, radius, 12)
        th_ = np.linspace(0, 2 * np.pi, 32)
        rr, tt = np.meshgrid(r_, th_)
        xd = cx + rr * np.cos(tt)
        yd = cy + rr * np.sin(tt)
        zd = np.full_like(xd, cz)
        ax.plot_surface(xd, yd, zd, color=color, alpha=alpha)

    def draw_robot_3d() -> None:
        ax.clear()
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_zlabel("z")
        ax.set_box_aspect([1, 1, 1])

        all_points = []

        # Robot links and joints (robot._current_joint_coordinates)
        coords = getattr(manager._robot, "_current_joint_coordinates", None)
        if coords is not None and len(coords) > 0:
            try:
                positions = np.asarray(coords).reshape(-1, 3)
            except (AttributeError, ValueError, TypeError):
                positions = np.empty((0, 3))
            if len(positions) >= 2:
                for (i, j) in LINK_PAIRS:
                    if i < len(positions) and j < len(positions):
                        ax.plot3D(
                            [positions[i, 0], positions[j, 0]],
                            [positions[i, 1], positions[j, 1]],
                            [positions[i, 2], positions[j, 2]],
                            "b-",
                            linewidth=2,
                        )
                ax.scatter(positions[:, 0], positions[:, 1], positions[:, 2], c="r", s=40, marker="o")
                all_points.append(positions)

        # Obstacles (robot._current_obstacles: CircleObstacleState, SelfObstacleState, etc.)
        obstacles = getattr(manager._robot, "_current_obstacles", None)
        if obstacles is not None:
            obs_list = obstacles if isinstance(obstacles, (list, tuple)) else [obstacles]
            for obs in obs_list:
                if getattr(obs, "position", None) is None or getattr(obs, "radius", None) is None:
                    continue
                try:
                    c = np.asarray(obs.position).flatten()[:3]
                    r = float(obs.radius)
                    has_axis = getattr(obs, "axis", None)
                    if has_axis is not None and has_axis == 2:
                        _draw_circle_obstacle(ax, c, r, color="orange", alpha=0.2)
                    else:
                        _draw_sphere(ax, c, r, color="orange", alpha=0.5)
                except (ValueError, TypeError):
                    pass

        # Planned path: traj is in joint space; use get_joint_coordinates, index 4 = end effector
        planner = getattr(manager._robot, "_planner", None)
        path_arr = None
        if planner is not None and getattr(planner, "is_planned", lambda: False)():
            get_traj = getattr(planner, "get_trajectory", None)
            if get_traj is not None:
                traj = get_traj()
                if traj:
                    robot = manager._robot
                    current_pos = getattr(robot, "_current_joint_state", None)
                    get_coords = getattr(robot, "get_joint_coordinates", None)
                    path_points = []
                    for _t, config in traj:
                        q = np.asarray(config).ravel()
                        n_j = getattr(robot, "_number_of_joints", 0)
                        if current_pos is not None and hasattr(current_pos, "position"):
                            full_q = np.asarray(current_pos.position, dtype=np.float64).ravel().copy()
                        else:
                            full_q = np.zeros(max(n_j, 1), dtype=np.float64)
                        n_fill = min(q.size, full_q.size)
                        if robot._home_count == 0:
                            full_q[:n_fill] = q[:n_fill]
                        else:
                            full_q[2:] = q
                        if get_coords is not None:
                            crd = get_coords(full_q)
                            if crd is not None and len(crd) > 4:
                                if robot._home_count == 0:
                                    path_points.append(np.asarray(crd[4]).flatten()[:3].astype(float))
                                else:
                                    path_points.append(np.asarray(crd[7]).flatten()[:3].astype(float))
                                continue
                        if q.size >= 3:
                            if robot._home_count == 0:
                                path_points.append(q[:3].astype(float))
                            else:
                                path_points.append(np.array([float(q[0]), float(q[1]), 0.0]))
                        elif q.size >= 2:
                            path_points.append(np.array([float(q[0]), float(q[1]), 0.0]))
                    if len(path_points) >= 1:
                        path_arr = np.array(path_points)
                        all_points.append(path_arr)
                        if len(path_arr) >= 2:
                            ax.plot3D(
                                path_arr[:, 0],
                                path_arr[:, 1],
                                path_arr[:, 2],
                                "g-",
                                linewidth=2.5,
                                label="path",
                            )
                        ax.scatter(
                            [path_arr[0, 0]],
                            [path_arr[0, 1]],
                            [path_arr[0, 2]],
                            c="lime",
                            s=150,
                            marker="o",
                            edgecolors="darkgreen",
                            linewidths=2,
                            label="start",
                            zorder=5,
                        )
                        if len(path_arr) > 2:
                            ax.scatter(
                                path_arr[1:-1, 0],
                                path_arr[1:-1, 1],
                                path_arr[1:-1, 2],
                                c="gold",
                                s=50,
                                marker="o",
                                edgecolors="orange",
                                linewidths=1,
                                zorder=4,
                            )
                        ax.scatter(
                            [path_arr[-1, 0]],
                            [path_arr[-1, 1]],
                            [path_arr[-1, 2]],
                            c="red",
                            s=150,
                            marker="*",
                            edgecolors="darkred",
                            linewidths=2,
                            label="goal",
                            zorder=5,
                        )
                        ax.legend(loc="upper left", fontsize=8)

        ax.set_xlim(-0.5, 0.5)
        ax.set_ylim(-0.5, 0.5)
        ax.set_zlim(-0.5, 0.5)

        canvas.draw_idle()

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

    def update() -> None:
        try:
            manager.update(status, obstacles)
            draw_robot_3d()
        except Exception:
            pass
        root.after(1, update)

    def control() -> None:
        try:
            control_state = manager.control(status)
            if control_state is not None:
                np.copyto(status.position, control_state.position)
                np.copyto(status.velocity, control_state.velocity)
                np.copyto(status.torque, control_state.torque)
        except Exception:
            pass
        root.after(10, control)

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
        position=np.array([-0.6, 1.4, 0.5, -1.0]),
        velocity=np.zeros(manager._robot._number_of_joints),
        torque=np.zeros(manager._robot._number_of_joints),
    )
    obstacles = None

    root.after(1, update)
    root.after(10, control)
    root.mainloop()


if __name__ == "__main__":
    if "--test" in sys.argv:
        sys.argv = [sys.argv[0]] + [a for a in sys.argv[1:] if a != "--test"]
        unittest.main()
    else:
        main_gui()
