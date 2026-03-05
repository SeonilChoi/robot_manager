"""Tests for RRT planner and RrtAlgorithm, plus 2-DOF trajectory visualization."""
import time
import unittest

import numpy as np

from robot_manager.core import JointState, ObstacleState
from robot_manager.planner import RrtPlanner
from robot_manager.utils import RrtAlgorithm
from robot_manager.utils.rrt import (
    quintic_time_scaling,
    joint_distance,
    steer,
    interpolate,
    STEP_SIZE,
    GOAL_THRESHOLD,
)

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


class TestQuinticTimeScaling(unittest.TestCase):
    def test_bounds(self):
        self.assertAlmostEqual(quintic_time_scaling(0.0), 0.0)
        self.assertAlmostEqual(quintic_time_scaling(1.0), 1.0)

    def test_clamping(self):
        self.assertAlmostEqual(quintic_time_scaling(-0.5), 0.0)
        self.assertAlmostEqual(quintic_time_scaling(1.5), 1.0)


class TestJointDistance(unittest.TestCase):
    def test_same_state(self):
        a = _make_joint_state([0.0, 0.0])
        self.assertAlmostEqual(joint_distance(a, a), 0.0)

    def test_different_state(self):
        a = _make_joint_state([0.0, 0.0])
        b = _make_joint_state([3.0, 4.0])
        self.assertAlmostEqual(joint_distance(a, b), 5.0)


class TestSteer(unittest.TestCase):
    def test_at_goal_within_step(self):
        a = _make_joint_state([0.0, 0.0])
        b = _make_joint_state([0.02, 0.0])
        out = steer(a, b, STEP_SIZE)
        np.testing.assert_array_almost_equal(out.position, b.position)

    def test_partial_step(self):
        a = _make_joint_state([0.0, 0.0])
        b = _make_joint_state([1.0, 0.0])
        out = steer(a, b, 0.3)
        np.testing.assert_array_almost_equal(out.position, [0.3, 0.0])


class TestInterpolate(unittest.TestCase):
    def test_endpoints(self):
        a = _make_joint_state([0.0, 0.0])
        b = _make_joint_state([1.0, 2.0])
        np.testing.assert_array_almost_equal(interpolate(a, b, 0.0).position, a.position)
        np.testing.assert_array_almost_equal(interpolate(a, b, 1.0).position, b.position)

    def test_midpoint(self):
        a = _make_joint_state([0.0, 0.0])
        b = _make_joint_state([1.0, 2.0])
        mid = interpolate(a, b, 0.5).position
        np.testing.assert_array_almost_equal(mid, [0.5, 1.0])


class TestRrtAlgorithm(unittest.TestCase):
    def test_same_start_goal_returns_success(self):
        rrt = RrtAlgorithm(seed=42)
        start = _make_joint_state([0.0, 0.0])
        goal = _make_joint_state([0.0, 0.0])
        success, traj = rrt.run(start, goal, None)
        self.assertTrue(success)
        self.assertGreaterEqual(len(traj), 1)

    def test_near_goal_returns_success(self):
        rrt = RrtAlgorithm(seed=123)
        start = _make_joint_state([0.0, 0.0])
        goal = _make_joint_state([0.03, 0.0])
        success, traj = rrt.run(start, goal, None)
        self.assertTrue(success)

    def test_mismatch_joint_size_returns_failure(self):
        rrt = RrtAlgorithm(seed=1)
        start = _make_joint_state([0.0, 0.0])
        goal = _make_joint_state([0.0, 0.0, 0.0])
        success, traj = rrt.run(start, goal, None)
        self.assertFalse(success)
        self.assertEqual(len(traj), 0)


class TestRrtPlanner(unittest.TestCase):
    def test_eval_after_plan(self):
        planner = RrtPlanner(seed=42)
        start = _make_joint_state([0.0, 0.0])
        goal = _make_joint_state([0.0, 0.0])
        success = planner.generate_trajectory(start, goal, None)
        self.assertTrue(success)
        planner._is_planned = True
        out0 = _make_joint_state([-1.0, -1.0])
        planner.eval(0.0, out0)
        np.testing.assert_array_almost_equal(out0.position, start.position)
        out1 = _make_joint_state([-1.0, -1.0])
        planner.eval(1.0, out1)
        np.testing.assert_array_almost_equal(out1.position, goal.position)

    def test_eval_without_plan_leaves_command_unchanged(self):
        planner = RrtPlanner()
        cmd = _make_joint_state([1.0, 2.0])
        planner.eval(0.5, cmd)
        np.testing.assert_array_almost_equal(cmd.position, [1.0, 2.0])

    def test_plan_triggers_run_and_generate_trajectory(self):
        planner = RrtPlanner(seed=42)
        start = _make_joint_state([0.0, 0.0])
        goal = _make_joint_state([0.0, 0.0])
        planner.plan(start, goal, None)
        deadline = time.monotonic() + 5.0
        while not planner.is_planned() and time.monotonic() < deadline:
            time.sleep(0.02)
        self.assertTrue(planner.is_planned())


# --- 2-DOF visualization helpers ---

def _fk_2dof(q1: float, q2: float, L1: float = 1.0, L2: float = 1.0):
    x_elbow = L1 * np.cos(q1)
    y_elbow = L1 * np.sin(q1)
    x_ee = x_elbow + L2 * np.cos(q1 + q2)
    y_ee = y_elbow + L2 * np.sin(q1 + q2)
    return (x_elbow, y_elbow), (x_ee, y_ee)


def _segment_circle_collision(a, b, center, radius):
    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)
    center = np.asarray(center, dtype=np.float64)
    ab = b - a
    ac = center - a
    denom = np.dot(ab, ab) + 1e-12
    t = np.dot(ac, ab) / denom
    t = np.clip(t, 0.0, 1.0)
    closest = a + t * ab
    return np.linalg.norm(center - closest) < radius


def _link_spheres_2dof(q: JointState, L1: float, L2: float, link_radius: float = 0.12):
    """Virtual collision spheres (2D circles) at midpoint of each link. Returns list of (center, radius)."""
    (x_elbow, y_elbow), (x_ee, y_ee) = _fk_2dof(q.position[0], q.position[1], L1, L2)
    base = np.array([0.0, 0.0])
    elbow = np.array([x_elbow, y_elbow])
    ee = np.array([x_ee, y_ee])
    mid1 = (base + elbow) / 2
    mid2 = (elbow + ee) / 2
    return [(mid1, link_radius), (mid2, link_radius)]


def _self_collision_spheres(link_spheres):
    """True if any two link spheres overlap (distance < r_i + r_j). link_spheres = [(center, radius), ...]."""
    n = len(link_spheres)
    for i in range(n):
        for j in range(i + 1, n):
            ci, ri = np.asarray(link_spheres[i][0]), link_spheres[i][1]
            cj, rj = np.asarray(link_spheres[j][0]), link_spheres[j][1]
            if np.linalg.norm(ci - cj) < ri + rj:
                return True
    return False


def _link_collision_2dof(q: JointState, obs_list, L1: float, L2: float, link_radius: float = 0.12):
    (x_elbow, y_elbow), (x_ee, y_ee) = _fk_2dof(q.position[0], q.position[1], L1, L2)
    base, elbow, ee = (0.0, 0.0), (x_elbow, y_elbow), (x_ee, y_ee)
    for obs in obs_list:
        if _segment_circle_collision(base, elbow, obs.position, obs.radius):
            return True
        if _segment_circle_collision(elbow, ee, obs.position, obs.radius):
            return True
    if _self_collision_spheres(_link_spheres_2dof(q, L1, L2, link_radius)):
        return True
    return False


@unittest.skipUnless(_HAS_MATPLOTLIB, "matplotlib not installed")
class TestRrtPlannerVisualization2Dof(unittest.TestCase):
    """Generate and visualize 2-DOF RRT trajectory with obstacles."""

    def test_visualize_2dof_trajectory(self):
        L1, L2 = 1.0, 1.0
        start = _make_joint_state([0.5, 0.5])
        goal = _make_joint_state([-0.4, 0.8])
        obstacles = [ObstacleState(position=np.array([2.0, 0.75]), radius=0.2)]

        def config_collision(q: JointState, obs_list):
            return _link_collision_2dof(q, obs_list, L1, L2, link_radius=0.12)

        def segment_collision(a: JointState, b: JointState, obs_list):
            for t in np.linspace(0.0, 1.0, 15):
                if t <= 0 or t >= 1:
                    continue
                q = interpolate(a, b, float(t))
                if _link_collision_2dof(q, obs_list, L1, L2, link_radius=0.12):
                    return True
            return False

        planner = RrtPlanner(seed=42)
        planner.set_joint_limits(
            min_positions=np.array([-np.pi, -np.pi]),
            max_positions=np.array([np.pi, np.pi]),
        )
        planner.set_collision_checker(config_fn=config_collision, segment_fn=segment_collision)
        success = planner.generate_trajectory(start, goal, obstacles)
        self.assertTrue(success, "RRT should find a path avoiding obstacles")
        planner._is_planned = True

        with planner._trajectory_mutex:
            traj = list(planner._trajectory)

        fig, ax = plt.subplots(1, 1, figsize=(8, 8))
        ax.set_aspect("equal")

        for obs in obstacles:
            circle = plt.Circle(
                obs.position, obs.radius, color="red", alpha=0.35, ec="darkred", linewidth=1.5
            )
            ax.add_patch(circle)
        ax.plot([], [], "o", color="red", alpha=0.35, markersize=12, label="Obstacles")

        xs_ee, ys_ee = [], []
        for _t, js in traj:
            _, (x_ee, y_ee) = _fk_2dof(js.position[0], js.position[1], L1, L2)
            xs_ee.append(x_ee)
            ys_ee.append(y_ee)
        ax.plot(xs_ee, ys_ee, "b-", linewidth=2, label="End-effector path")

        indices = [0, len(traj) // 2, len(traj) - 1] if len(traj) >= 3 else list(range(len(traj)))
        for idx in indices:
            _t, js = traj[idx]
            q1, q2 = js.position[0], js.position[1]
            (x_elbow, y_elbow), (x_ee, y_ee) = _fk_2dof(q1, q2, L1, L2)
            if idx == 0:
                ax.plot([0, x_elbow, x_ee], [0, y_elbow, y_ee], "g-o", linewidth=2, label="Start")
            elif idx == len(traj) - 1:
                ax.plot([0, x_elbow, x_ee], [0, y_elbow, y_ee], "m-o", linewidth=2, label="Goal")
            else:
                ax.plot([0, x_elbow, x_ee], [0, y_elbow, y_ee], "o-", color="orange", alpha=0.8)

        ax.plot(0, 0, "ko", markersize=10, label="Base")
        for (center, r) in _link_spheres_2dof(traj[0][1], L1, L2, link_radius=0.12):
            circle = plt.Circle(center, r, color="cyan", alpha=0.2, ec="blue", linewidth=0.8)
            ax.add_patch(circle)
        ax.plot([], [], "o", color="cyan", alpha=0.2, markersize=8, label="Link spheres (self-collision)")
        ax.legend(loc="upper right")
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_title("2-DOF RRT trajectory with obstacles and self-collision")
        ax.grid(True, alpha=0.3)
        out_path = __file__.replace("rrt_planner_test.py", "rrt_2dof_trajectory.png")
        fig.savefig(out_path, dpi=120, bbox_inches="tight")
        plt.close(fig)
        self.assertGreater(len(traj), 0)


# --- 3-DOF spatial arm (3D) helpers ---

def _fk_3dof(q1: float, q2: float, q3: float, L1: float = 1.0, L2: float = 1.0, L3: float = 1.0):
    """Forward kinematics for 3-R spatial arm: q1 about Z, q2/q3 about Y. Returns (base, shoulder, elbow, ee) as (3,) arrays."""
    base = np.array([0.0, 0.0, 0.0])
    shoulder = np.array([L1 * np.cos(q1), L1 * np.sin(q1), 0.0])
    elbow = shoulder + np.array([
        L2 * np.cos(q1) * np.cos(q2),
        L2 * np.sin(q1) * np.cos(q2),
        L2 * np.sin(q2),
    ])
    c23 = np.cos(q2 + q3)
    s23 = np.sin(q2 + q3)
    ee = elbow + np.array([
        L3 * np.cos(q1) * c23,
        L3 * np.sin(q1) * c23,
        L3 * s23,
    ])
    return base, shoulder, elbow, ee


def _segment_sphere_collision(a, b, center, radius):
    """True if segment a-b intersects sphere at center with given radius (3D)."""
    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)
    center = np.asarray(center, dtype=np.float64)
    ab = b - a
    ac = center - a
    denom = np.dot(ab, ab) + 1e-12
    t = np.dot(ac, ab) / denom
    t = np.clip(t, 0.0, 1.0)
    closest = a + t * ab
    return np.linalg.norm(center - closest) < radius


def _link_spheres_3dof(q: JointState, L1: float, L2: float, L3: float, link_radius: float = 0.12):
    """Virtual collision spheres at midpoint of each link (3-DOF). Returns list of (center, radius)."""
    base, shoulder, elbow, ee = _fk_3dof(q.position[0], q.position[1], q.position[2], L1, L2, L3)
    mid1 = (base + shoulder) / 2
    mid2 = (shoulder + elbow) / 2
    mid3 = (elbow + ee) / 2
    return [(mid1, link_radius), (mid2, link_radius), (mid3, link_radius)]


def _link_collision_3dof(q: JointState, obs_list, L1: float, L2: float, L3: float, link_radius: float = 0.12):
    """True if any link collides with an obstacle sphere or with another link (self-collision)."""
    base, shoulder, elbow, ee = _fk_3dof(q.position[0], q.position[1], q.position[2], L1, L2, L3)
    for obs in obs_list:
        c = obs.position if obs.position.size >= 3 else np.array([obs.position[0], obs.position[1], 0.0])
        r = obs.radius
        if _segment_sphere_collision(base, shoulder, c, r):
            return True
        if _segment_sphere_collision(shoulder, elbow, c, r):
            return True
        if _segment_sphere_collision(elbow, ee, c, r):
            return True
    if _self_collision_spheres(_link_spheres_3dof(q, L1, L2, L3, link_radius)):
        return True
    return False


def _draw_sphere(ax, center, radius, color="red", alpha=0.35):
    """Draw a sphere in 3D on ax (Axes3D)."""
    u = np.linspace(0, 2 * np.pi, 16)
    v = np.linspace(0, np.pi, 12)
    x = center[0] + radius * np.outer(np.cos(u), np.sin(v))
    y = center[1] + radius * np.outer(np.sin(u), np.sin(v))
    z = center[2] + radius * np.outer(np.ones_like(u), np.cos(v))
    ax.plot_surface(x, y, z, color=color, alpha=alpha)


@unittest.skipUnless(_HAS_MATPLOTLIB, "matplotlib not installed")
class TestRrtPlannerVisualization3Dof(unittest.TestCase):
    """Generate and visualize 3-DOF spatial RRT trajectory in 3D."""

    def test_visualize_3dof_spatial_trajectory(self):
        L1, L2, L3 = 1.0, 1.0, 1.0
        start = _make_joint_state([0.3, 0.4, 0.2])
        goal = _make_joint_state([-0.2, 0.6, 0.5])
        obstacles = [
            ObstacleState(position=np.array([0.8, 0.5, 0.3]), radius=0.2),
        ]

        link_radius = 0.12

        def config_collision(q: JointState, obs_list):
            return _link_collision_3dof(q, obs_list, L1, L2, L3, link_radius=link_radius)

        def segment_collision(a: JointState, b: JointState, obs_list):
            for t in np.linspace(0.0, 1.0, 15):
                if t <= 0 or t >= 1:
                    continue
                q = interpolate(a, b, float(t))
                if _link_collision_3dof(q, obs_list, L1, L2, L3, link_radius=link_radius):
                    return True
            return False

        planner = RrtPlanner(seed=42)
        planner.set_joint_limits(
            min_positions=np.array([-np.pi, -np.pi, -np.pi]),
            max_positions=np.array([np.pi, np.pi, np.pi]),
        )
        planner.set_collision_checker(config_fn=config_collision, segment_fn=segment_collision)
        success = planner.generate_trajectory(start, goal, obstacles)
        self.assertTrue(success, "RRT should find a path for 3-DOF spatial arm")
        planner._is_planned = True

        with planner._trajectory_mutex:
            traj = list(planner._trajectory)

        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection="3d")

        for obs in obstacles:
            c = obs.position if obs.position.size >= 3 else np.array([obs.position[0], obs.position[1], 0.0])
            _draw_sphere(ax, c, obs.radius, color="red", alpha=0.35)

        xs_ee, ys_ee, zs_ee = [], [], []
        for _t, js in traj:
            _, _, _, ee = _fk_3dof(js.position[0], js.position[1], js.position[2], L1, L2, L3)
            xs_ee.append(ee[0])
            ys_ee.append(ee[1])
            zs_ee.append(ee[2])
        ax.plot(xs_ee, ys_ee, zs_ee, "b-", linewidth=2, label="End-effector path")

        indices = [0, len(traj) // 2, len(traj) - 1] if len(traj) >= 3 else list(range(len(traj)))
        for idx in indices:
            _t, js = traj[idx]
            base, shoulder, elbow, ee = _fk_3dof(js.position[0], js.position[1], js.position[2], L1, L2, L3)
            ax.plot(
                [base[0], shoulder[0], elbow[0], ee[0]],
                [base[1], shoulder[1], elbow[1], ee[1]],
                [base[2], shoulder[2], elbow[2], ee[2]],
                "o-",
                color="green" if idx == 0 else ("magenta" if idx == len(traj) - 1 else "orange"),
                linewidth=2 if idx in (0, len(traj) - 1) else 1,
            )
        ax.scatter([0], [0], [0], color="black", s=80, label="Base")
        for (center, r) in _link_spheres_3dof(traj[0][1], L1, L2, L3, link_radius=0.12):
            _draw_sphere(ax, center, r, color="cyan", alpha=0.2)
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_zlabel("z")
        ax.legend(loc="upper right")
        ax.set_title("3-DOF spatial RRT trajectory with self-collision")
        out_path = __file__.replace("rrt_planner_test.py", "rrt_3dof_trajectory.png")
        fig.savefig(out_path, dpi=120, bbox_inches="tight")
        plt.close(fig)
        self.assertGreater(len(traj), 0)


if __name__ == "__main__":
    unittest.main()
