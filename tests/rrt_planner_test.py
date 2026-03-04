"""Tests for RRT planner and RrtAlgorithm, plus 2-DOF trajectory visualization."""
import time
import unittest

import numpy as np

from robot_manager.core.planner import JointState, Obstacle
from robot_manager.planner import RrtPlanner, RrtAlgorithm
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
    _HAS_MATPLOTLIB = True
except ImportError:
    _HAS_MATPLOTLIB = False


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


def _link_collision_2dof(q: JointState, obs_list, L1: float, L2: float):
    (x_elbow, y_elbow), (x_ee, y_ee) = _fk_2dof(q.position[0], q.position[1], L1, L2)
    base, elbow, ee = (0.0, 0.0), (x_elbow, y_elbow), (x_ee, y_ee)
    for obs in obs_list:
        if _segment_circle_collision(base, elbow, obs.position, obs.radius):
            return True
        if _segment_circle_collision(elbow, ee, obs.position, obs.radius):
            return True
    return False


@unittest.skipUnless(_HAS_MATPLOTLIB, "matplotlib not installed")
class TestRrtPlannerVisualization2Dof(unittest.TestCase):
    """Generate and visualize 2-DOF RRT trajectory with obstacles."""

    def test_visualize_2dof_trajectory(self):
        L1, L2 = 1.0, 1.0
        start = _make_joint_state([0.5, 0.5])
        goal = _make_joint_state([-0.4, 0.8])
        obstacles = [Obstacle(position=np.array([2.0, 0.75]), radius=0.2)]

        def config_collision(q: JointState, obs_list):
            return _link_collision_2dof(q, obs_list, L1, L2)

        def segment_collision(a: JointState, b: JointState, obs_list):
            for t in np.linspace(0.0, 1.0, 15):
                if t <= 0 or t >= 1:
                    continue
                q = interpolate(a, b, float(t))
                if _link_collision_2dof(q, obs_list, L1, L2):
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
        ax.legend(loc="upper right")
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_title("2-DOF RRT trajectory with obstacles")
        ax.grid(True, alpha=0.3)
        out_path = __file__.replace("rrt_planner_test.py", "rrt_2dof_trajectory.png")
        fig.savefig(out_path, dpi=120, bbox_inches="tight")
        plt.close(fig)
        self.assertGreater(len(traj), 0)


if __name__ == "__main__":
    unittest.main()
