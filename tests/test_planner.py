"""Planner tests: RrtAlgorithm, RrtPlanner, and visualization for each config space (joint, pose.position, velocity.linear)."""
from __future__ import annotations

import os
import time
import unittest

import numpy as np

from robot_manager.core import JointState, ObstacleState
from robot_manager.planner import RrtPlanner
from robot_manager.utils import RrtAlgorithm

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    _HAS_MATPLOTLIB = True
except ImportError:
    _HAS_MATPLOTLIB = False
    Axes3D = None


def _make_joint_state(position):
    pos = np.atleast_1d(np.asarray(position, dtype=np.float64))
    return JointState(
        id=np.arange(pos.size, dtype=np.float64),
        position=pos.copy(),
        velocity=np.zeros_like(pos),
        torque=np.zeros_like(pos),
    )


def _point_in_sphere(point: np.ndarray, center: np.ndarray, radius: float) -> bool:
    return float(np.linalg.norm(np.asarray(point) - np.asarray(center))) < radius


def _segment_intersects_sphere(a: np.ndarray, b: np.ndarray, center: np.ndarray, radius: float) -> bool:
    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)
    center = np.asarray(center, dtype=np.float64)
    ab = b - a
    ac = center - a
    denom = np.dot(ab, ab) + 1e-12
    t = np.clip(np.dot(ac, ab) / denom, 0.0, 1.0)
    closest = a + t * ab
    return np.linalg.norm(center - closest) < radius


def _config_collision_spheres(config: np.ndarray, obstacle_state) -> bool:
    if obstacle_state is None:
        return False
    config = np.asarray(config, dtype=np.float64).ravel()
    for obs in obstacle_state:
        c, r = np.asarray(obs.position).ravel(), obs.radius
        if _point_in_sphere(config, c, r):
            return True
    return False


def _segment_collision_spheres(config_a: np.ndarray, config_b: np.ndarray, obstacle_state) -> bool:
    if obstacle_state is None:
        return False
    a = np.asarray(config_a, dtype=np.float64).ravel()
    b = np.asarray(config_b, dtype=np.float64).ravel()
    for obs in obstacle_state:
        c, r = np.asarray(obs.position).ravel(), obs.radius
        if _segment_intersects_sphere(a, b, c, r):
            return True
    return False


def _visualizations_dir() -> str:
    """Return path to tests/visualizations; create if missing."""
    base = os.path.dirname(os.path.abspath(__file__))
    out = os.path.join(base, "visualizations")
    os.makedirs(out, exist_ok=True)
    return out


# ----- RrtAlgorithm -----


class TestRrtAlgorithm(unittest.TestCase):
    def test_same_start_goal_success(self):
        rrt = RrtAlgorithm(seed=42)
        start = _make_joint_state([0.0, 0.0])
        goal = _make_joint_state([0.0, 0.0])
        success, traj = rrt.run(start, goal, None)
        self.assertTrue(success)
        self.assertGreaterEqual(len(traj), 1)

    def test_mismatch_size_returns_failure(self):
        rrt = RrtAlgorithm(seed=1)
        start = _make_joint_state([0.0, 0.0])
        goal = _make_joint_state([0.0, 0.0, 0.0])
        success, traj = rrt.run(start, goal, None)
        self.assertFalse(success)
        self.assertEqual(len(traj), 0)


# ----- RrtPlanner -----


class TestRrtPlanner(unittest.TestCase):
    def test_eval_after_plan(self):
        planner = RrtPlanner(seed=42)
        start = np.array([0.0, 0.0])
        goal = np.array([0.0, 0.0])
        success = planner.generate_trajectory(start, goal, None)
        self.assertTrue(success)
        planner._is_planned = True
        out0 = planner.eval(0.0)
        self.assertIsNotNone(out0)
        np.testing.assert_array_almost_equal(out0, start)
        out1 = planner.eval(1.0)
        self.assertIsNotNone(out1)
        np.testing.assert_array_almost_equal(out1, goal)

    def test_eval_without_plan_returns_none(self):
        planner = RrtPlanner()
        out = planner.eval(0.5)
        self.assertIsNone(out)

    def test_plan_triggers_run_and_generate_trajectory(self):
        planner = RrtPlanner(seed=42)
        start = np.array([0.0, 0.0])
        goal = np.array([0.0, 0.0])
        planner.plan(start, goal, None)
        deadline = time.monotonic() + 5.0
        while not planner.is_planned() and time.monotonic() < deadline:
            time.sleep(0.02)
        self.assertTrue(planner.is_planned())


# ----- Visualize each configuration space (required) -----


@unittest.skipUnless(_HAS_MATPLOTLIB, "matplotlib not installed")
class TestVisualizeConfigSpaces(unittest.TestCase):
    """Plot and save RRT path for each config space: joint, pose.position, velocity.linear."""

    def test_visualize_joint_space(self):
        """joint_state.position space: 2D path with obstacle."""
        planner = RrtPlanner(seed=123)
        planner.set_joint_limits(
            min_positions=np.array([-np.pi, -np.pi]),
            max_positions=np.array([np.pi, np.pi]),
        )
        planner.set_collision_checker(
            collision_fn=_config_collision_spheres,
            segment_fn=_segment_collision_spheres,
        )
        start = np.array([0.0, 0.0])
        goal = np.array([1.0, 1.0])
        obstacles = [ObstacleState(position=np.array([0.5, 0.5]), radius=0.35, zaxis=True)]
        success = planner.generate_trajectory(start, goal, obstacle_state=obstacles)
        self.assertTrue(success)

        path = []
        for i in range(81):
            c = planner.eval(i / 80.0)
            if c is not None:
                path.append(c)
        path = np.array(path)

        fig, ax = plt.subplots(1, 1, figsize=(7, 7))
        ax.set_aspect("equal")
        for obs in obstacles:
            circle = plt.Circle(obs.position, obs.radius, color="red", alpha=0.35, ec="darkred", linewidth=1.5)
            ax.add_patch(circle)
        ax.plot(path[:, 0], path[:, 1], "b-", linewidth=2, label="RRT path (joint space)")
        ax.plot(start[0], start[1], "go", markersize=12, label="Start")
        ax.plot(goal[0], goal[1], "ms", markersize=12, label="Goal")
        ax.set_xlabel("Joint 1 (rad)")
        ax.set_ylabel("Joint 2 (rad)")
        ax.set_title("RrtPlanner: joint_state.position space")
        ax.legend(loc="upper right")
        ax.grid(True, alpha=0.3)
        fig.savefig(os.path.join(_visualizations_dir(), "config_space_joint.png"), dpi=120, bbox_inches="tight")
        plt.close(fig)
        self.assertGreater(len(path), 2)

    def test_visualize_pose_position_space(self):
        """pose.position space: 3D Cartesian path with obstacle."""
        planner = RrtPlanner(seed=100)
        planner.set_bounds(
            min_bounds=np.array([-0.5, -0.5, -0.5]),
            max_bounds=np.array([1.5, 0.5, 0.5]),
        )
        planner.set_collision_checker(
            collision_fn=_config_collision_spheres,
            segment_fn=_segment_collision_spheres,
        )
        start = np.array([0.0, 0.0, 0.0])
        goal = np.array([1.0, 0.0, 0.0])
        obstacles = [ObstacleState(position=np.array([0.5, 0.0, 0.0]), radius=0.25, zaxis=True)]
        success = planner.generate_trajectory(start, goal, obstacle_state=obstacles)
        self.assertTrue(success)

        path = []
        for i in range(81):
            c = planner.eval(i / 80.0)
            if c is not None:
                path.append(c)
        path = np.array(path)

        fig = plt.figure(figsize=(8, 6))
        ax = fig.add_subplot(111, projection="3d")
        ax.plot(path[:, 0], path[:, 1], path[:, 2], "b-", linewidth=2, label="RRT path")
        ax.scatter([start[0]], [start[1]], [start[2]], color="green", s=80, label="Start")
        ax.scatter([goal[0]], [goal[1]], [goal[2]], color="magenta", s=80, label="Goal")
        u = np.linspace(0, 2 * np.pi, 20)
        v = np.linspace(0, np.pi, 15)
        for obs in obstacles:
            cx, cy, cz = obs.position[0], obs.position[1], obs.position[2]
            x = cx + obs.radius * np.outer(np.cos(u), np.sin(v))
            y = cy + obs.radius * np.outer(np.sin(u), np.sin(v))
            z = cz + obs.radius * np.outer(np.ones_like(u), np.cos(v))
            ax.plot_surface(x, y, z, color="red", alpha=0.3)
        ax.set_xlabel("x (pose.position)")
        ax.set_ylabel("y")
        ax.set_zlabel("z")
        ax.set_title("RrtPlanner: pose.position space")
        ax.legend(loc="upper right")
        fig.savefig(os.path.join(_visualizations_dir(), "config_space_pose_position.png"), dpi=120, bbox_inches="tight")
        plt.close(fig)
        self.assertGreater(len(path), 2)

    def test_visualize_velocity_linear_space(self):
        """velocity.linear space: 3D velocity path with obstacle."""
        planner = RrtPlanner(seed=200)
        planner.set_bounds(
            min_bounds=np.array([-1.0, -1.0, -1.0]),
            max_bounds=np.array([1.0, 1.0, 1.0]),
        )
        planner.set_collision_checker(
            collision_fn=_config_collision_spheres,
            segment_fn=_segment_collision_spheres,
        )
        start = np.array([0.0, 0.0, 0.0])
        goal = np.array([1.0, 0.0, 0.0])
        obstacles = [ObstacleState(position=np.array([0.5, 0.0, 0.0]), radius=0.3, zaxis=True)]
        success = planner.generate_trajectory(start, goal, obstacle_state=obstacles)
        self.assertTrue(success)

        path = []
        for i in range(81):
            c = planner.eval(i / 80.0)
            if c is not None:
                path.append(c)
        path = np.array(path)

        fig = plt.figure(figsize=(8, 6))
        ax = fig.add_subplot(111, projection="3d")
        ax.plot(path[:, 0], path[:, 1], path[:, 2], "b-", linewidth=2, label="RRT path (velocity)")
        ax.scatter([start[0]], [start[1]], [start[2]], color="green", s=80, label="Start")
        ax.scatter([goal[0]], [goal[1]], [goal[2]], color="magenta", s=80, label="Goal")
        for obs in obstacles:
            cx, cy, cz = obs.position[0], obs.position[1], obs.position[2]
            u = np.linspace(0, 2 * np.pi, 20)
            v = np.linspace(0, np.pi, 15)
            x = cx + obs.radius * np.outer(np.cos(u), np.sin(v))
            y = cy + obs.radius * np.outer(np.sin(u), np.sin(v))
            z = cz + obs.radius * np.outer(np.ones_like(u), np.cos(v))
            ax.plot_surface(x, y, z, color="red", alpha=0.3)
        ax.set_xlabel("vx (velocity.linear)")
        ax.set_ylabel("vy")
        ax.set_zlabel("vz")
        ax.set_title("RrtPlanner: velocity.linear space")
        ax.legend(loc="upper right")
        fig.savefig(os.path.join(_visualizations_dir(), "config_space_velocity_linear.png"), dpi=120, bbox_inches="tight")
        plt.close(fig)
        self.assertGreater(len(path), 2)


if __name__ == "__main__":
    unittest.main()
