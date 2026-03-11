# Planner

Path planning from **current** to **target** in a **background thread**. Config space is generic (`np.ndarray`). Sample the trajectory with `eval(progress)`.

```mermaid
flowchart LR
  plan[plan] --> worker[worker thread]
  worker --> G["_generate_trajectory"]
  G --> traj([trajectory])
  traj --> E[eval]
  E --> config[config]
```

---

## Overview

- **Purpose:** Non-blocking planning in a worker thread; main thread calls `plan()` then `eval(progress)` to get config along the path. Quintic time scaling and interpolation used for smooth sampling.
- **Stack:** `core.Planner` (ABC) → `RrtPlanner` (concrete, uses `utils.RrtAlgorithm`).

---

## Class: RrtPlanner

- **Module:** `planner/rrt_planner.py`
- **Inherits:** `core.Planner`
- **Role:** Runs RRT in the worker thread; stores trajectory as list of `(progress, config)`; exposes `eval(progress)` with quintic scaling and interpolation.
- **Constructor:** `__init__(seed=None)` — creates `RrtAlgorithm`, trajectory list, and lock. No threading inside RrtAlgorithm.
- **set_bounds(min_bounds, max_bounds)** — Set sampling bounds (passed to RrtAlgorithm). Call before planning.
- **set_collision_checker(collision_fn, segment_fn)** — Optional point and segment collision checkers. Required for obstacle avoidance.
- **plan(current, target, obstacle_state)** — Inherited. Enqueues plan; worker calls `_generate_trajectory` which runs `RrtAlgorithm.run(start, goal, obstacles)`.
- **_generate_trajectory(current_state, target_state, obstacle_state) -> bool** — Runs RRT, stores trajectory, sets `_is_planned`. Returns True if path found.
- **eval(progress) -> np.ndarray | None** — Quintic time scaling then linear interpolation between waypoints. Returns None if not planned or empty trajectory.
- **get_trajectory()** — Returns copy of `[(t, config), ...]`; empty if not planned.
- **reset()** — Clears trajectory and resets planner state.

---

## Types and callbacks

- **CollisionFn:** `(config, obstacle_list) -> bool` — True if config is in collision.
- **SegmentCollisionFn:** `(config_a, config_b, obstacle_list) -> bool` — True if segment intersects an obstacle.
- Obstacles: `SphereObstacleState`, `CircleObstacleState`, `SelfObstacleState` (from `types`).
