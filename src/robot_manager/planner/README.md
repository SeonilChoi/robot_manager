# Planner

Motion planning in **joint space**: compute a path from current joint angles to a target, with optional obstacle and self-collision avoidance. Planning runs in a **background thread**; you request a path with `plan()` and sample it with `eval(progress)`.

---

## Architecture (visual)

```
┌─────────────────────────────────────────────────────────────────────────────┐
│  Your code                                                                   │
│    plan(start, goal, obstacles)  ──►  returns immediately                  │
│    while not is_planned(): sleep(...)                                        │
│    eval(progress, joint_command) ──►  fill joint_command along path         │
└─────────────────────────────────────────────────────────────────────────────┘
                    │                                    │
                    ▼                                    ▼
┌───────────────────────────────────┐    ┌───────────────────────────────────┐
│  Planner (core.planner)           │    │  RrtPlanner (this package)         │
│  • Worker thread runs _run()       │    │  • Subclass of Planner             │
│  • _run() calls                    │    │  • generate_trajectory() calls   │
│    generate_trajectory()           │◄───│    _rrt.run() and stores result   │
│  • Sets _is_planned when done      │    │  • eval() uses stored trajectory  │
└───────────────────────────────────┘    │    + quintic scaling + interpolate│
                    │                    └───────────────────────────────────┘
                    │                                    │
                    │                    ┌───────────────────────────────────┐
                    │                    │  RrtAlgorithm (utils.rrt)          │
                    └──────────────────►│  • run(start, goal, obstacle_state)│
                         (abstract)      │  • Returns (success, trajectory)  │
                                         │  • No threading                    │
                                         └───────────────────────────────────┘
```

**Data flow:**

```
  plan(current, target, obstacle_state)
       │
       ▼
  [Store current/target/obstacle; set _is_running=True; notify worker]
       │
       ▼
  Worker: _run()  ──►  generate_trajectory(current, target, obstacle)
                              │
                              ▼
                       RrtAlgorithm.run()  ──►  (success, [(t0, js0), (t1, js1), ...])
                              │
                              ▼
                       Store trajectory; _is_planned = success
       │
       ▼
  eval(progress, joint_command)  ──►  quintic_time_scaling(progress), find segment, interpolate
       │
       ▼
  joint_command.position / velocity / torque filled in-place
```

---

## Module layout

| Module | Contents |
|--------|----------|
| `robot_manager.core.planner` | `Planner` (ABC), `JointState`, `Obstacle` |
| `robot_manager.utils.rrt` | `RrtAlgorithm`, `run()`, helpers (`quintic_time_scaling`, `interpolate`, `steer`, `joint_distance`) |
| `robot_manager.planner` | `RrtPlanner` (Planner + RrtAlgorithm), re-exports `RrtAlgorithm` |

---

## Function reference

### Base class: `Planner` (core.planner)

Abstract base. Subclass and implement `eval()` and `generate_trajectory()`.

---

#### `plan(current_state, target_state, obstacle_state)`

**Purpose:** Request a new trajectory from current to target. Returns immediately; a worker thread computes the path.

**Parameters:**

| Parameter | Type | Description |
|-----------|------|-------------|
| `current_state` | `JointState` | Start joint configuration (position, velocity, torque). |
| `target_state` | `JointState` | Goal joint configuration. |
| `obstacle_state` | `Any` | Passed to `generate_trajectory` (e.g. list of obstacles). Can be `None`. |

**Behavior:**

- If the worker is already busy (`_is_running`), returns without changing the request.
- Otherwise stores the three arguments, sets `_is_planned = False`, sets `_is_running = True`, and notifies the worker. The worker will call `generate_trajectory(current_state, target_state, obstacle_state)` and then set `_is_planned` to the return value.

**Usage:** Call whenever you want a new path. Poll `is_planned()` until `True` (or timeout) before using `eval()`.

---

#### `eval(progress, joint_command)`

**Purpose:** Evaluate the planned trajectory at a given progress and write the result into `joint_command` (in-place).

**Parameters:**

| Parameter | Type | Description |
|-----------|------|-------------|
| `progress` | `float` | Path parameter in [0, 1]. 0 = start, 1 = goal. Often time-scaled (e.g. quintic) inside the implementation. |
| `joint_command` | `JointState` | Output. Its `position`, `velocity`, and `torque` arrays are overwritten. |

**Behavior:**

- If no plan is ready (`_is_planned` is False) or trajectory is empty, returns without writing.
- Otherwise finds the segment containing the (possibly time-scaled) progress and linearly interpolates between waypoints; writes position/velocity/torque into `joint_command`.

**Usage:** In your control loop, call e.g. `eval(t, joint_command)` with `t` from 0 to 1 and send `joint_command` to the robot.

---

#### `is_planned() -> bool`

**Purpose:** Report whether the last planning request has finished and produced a valid trajectory.

**Returns:** `True` if `generate_trajectory()` completed and returned `True`; `False` otherwise (not yet done or failed).

**Usage:** Poll after `plan()` to know when you can safely call `eval()`.

---

#### `request_stop()`

**Purpose:** Ask the worker thread to exit. Call before joining the planner thread (e.g. in a destructor or shutdown path).

**Behavior:** Sets `_stop_requested = True` and notifies the worker. The worker’s `_run()` loop will break on the next wake-up.

**Usage:** Call when shutting down; then join `_planner_thread` so the process can exit cleanly.

---

#### `generate_trajectory(current_state, target_state, obstacle_state) -> bool` *(abstract)*

**Purpose:** Compute a trajectory from current to target. Called by the worker thread; implement in subclasses.

**Parameters:** Same as `plan()` (current, target, obstacle_state).

**Returns:** `True` if a trajectory was found and stored; `False` otherwise.

**Usage:** You do not call this directly; the worker calls it after `plan()`.

---

#### `_run()` *(internal)*

**Purpose:** Worker loop: wait for work or stop, then call `generate_trajectory()` and set `_is_planned`.

**Behavior:** Runs in a separate thread. Waits on a condition until `_stop_requested` or `_is_running`. If stop, exits. If running, loads current/target/obstacle, clears `_is_running`, calls `generate_trajectory(...)`, and sets `_is_planned` to its return value. Repeats.

---

### RrtPlanner (this package)

Concrete planner that uses `RrtAlgorithm` inside the worker.

---

#### `RrtPlanner(dt=0.01, seed=None)`

**Parameters:**

| Parameter | Type | Description |
|-----------|------|-------------|
| `dt` | `float` | Time step (e.g. for logging); default `0.01`. |
| `seed` | `int \| None` | Random seed for RRT; `None` for non-deterministic. |

---

#### `set_joint_limits(min_positions, max_positions)`

**Purpose:** Set joint bounds used by RRT when sampling random configurations.

**Parameters:**

| Parameter | Type | Description |
|-----------|------|-------------|
| `min_positions` | `np.ndarray` | Lower bound per joint (length = number of joints). |
| `max_positions` | `np.ndarray` | Upper bound per joint (same length). |

**Usage:** Call before `plan()` if you want to restrict the sampling range. If not set, RRT uses default bounds (e.g. ±π per joint).

---

#### `set_collision_checker(config_fn=None, segment_fn=None)`

**Purpose:** Register optional collision checkers for environment (and optionally self-collision inside the callbacks).

**Parameters:**

| Parameter | Type | Description |
|-----------|------|-------------|
| `config_fn` | `(JointState, Any) -> bool` or `None` | Called as `config_fn(q, obstacle_state)`. Return `True` if configuration `q` is in collision. |
| `segment_fn` | `(JointState, JointState, Any) -> bool` or `None` | Called as `segment_fn(a, b, obstacle_state)`. Return `True` if the segment between `a` and `b` is in collision. If `None`, the algorithm interpolates between `a` and `b` and uses `config_fn` only. |

**Usage:** Call before `plan()`; pass your obstacle representation as `obstacle_state` in `plan()`. The same `obstacle_state` is passed to both callbacks.

---

#### `generate_trajectory(current_state, target_state, obstacle_state) -> bool`

**Purpose:** Called by the base worker. Runs `RrtAlgorithm.run(...)`, stores the returned trajectory, and returns success.

**Parameters:** Same as `plan()`.

**Returns:** `True` if RRT found a path; `False` otherwise.

---

#### `eval(progress, joint_command)`

**Purpose:** Same contract as base `Planner.eval`. Uses the stored RRT trajectory, applies quintic time scaling to `progress`, finds the segment, and interpolates into `joint_command`.

**Parameters:** Same as base `eval()`.

---

### RrtAlgorithm (utils.rrt)

Pure RRT logic; no threading. Use directly in scripts or tests, or via `RrtPlanner`.

---

#### `RrtAlgorithm(*, step_size=0.1, goal_bias=0.1, goal_threshold=0.05, max_iterations=5000, interp_steps=10, seed=None)`

**Parameters:**

| Parameter | Type | Description |
|-----------|------|-------------|
| `step_size` | `float` | Max step length when extending the tree. |
| `goal_bias` | `float` | Probability of sampling the goal (0–1). |
| `goal_threshold` | `float` | Distance to goal below which the path is considered reached. |
| `max_iterations` | `int` | Max RRT iterations per run. |
| `interp_steps` | `int` | Number of steps for default segment collision check when `segment_fn` is not set. |
| `seed` | `int \| None` | Random seed; `None` for non-deterministic. |

---

#### `set_joint_limits(min_positions, max_positions)`

Same semantics as `RrtPlanner.set_joint_limits`; configures sampling bounds.

---

#### `set_collision_checker(config_fn=None, segment_fn=None)`

Same semantics as `RrtPlanner.set_collision_checker`.

---

#### `run(start, goal, obstacle_state) -> (bool, list[(float, JointState)])`

**Purpose:** Run RRT from start to goal. No threading; blocks until done.

**Parameters:**

| Parameter | Type | Description |
|-----------|------|-------------|
| `start` | `JointState` | Start configuration. |
| `goal` | `JointState` | Goal configuration. |
| `obstacle_state` | `Any` | Passed to `config_fn` and `segment_fn`. |

**Returns:**

- **success:** `True` if a path was found; `False` otherwise (e.g. joint size mismatch, max iterations, or collision).
- **trajectory:** List of `(t, JointState)` with `t` in [0, 1]. Empty if not success.

**Usage:** Use when you want RRT without the planner thread (e.g. tests or one-off planning).

---

### Helper functions (utils.rrt)

| Function | Signature | Description |
|----------|-----------|-------------|
| `quintic_time_scaling` | `(t: float) -> float` | Maps [0,1] to [0,1] with zero velocity at 0 and 1. Used in `eval()`. |
| `joint_distance` | `(a: JointState, b: JointState) -> float` | Euclidean distance between `a.position` and `b.position`. |
| `steer` | `(from_state, toward_state, step_size) -> JointState` | New state from `from_state` toward `toward_state` by at most `step_size`. |
| `interpolate` | `(a: JointState, b: JointState, t: float) -> JointState` | Linear interpolation: `(1-t)*a + t*b` for positions. |

---

## Types (core.planner)

| Type | Fields | Description |
|------|--------|-------------|
| `JointState` | `id`, `position`, `velocity`, `torque` (all `np.ndarray`) | One joint configuration; `position` is the main one for RRT. |
| `Obstacle` | `position` (`np.ndarray`), `radius` (`float`) | Simple sphere obstacle; use in your `obstacle_state` (e.g. list of `Obstacle`) and check in `config_fn` / `segment_fn`. |

---

## Minimal example

```python
import time
import numpy as np
from robot_manager.planner import RrtPlanner
from robot_manager.core.planner import JointState

planner = RrtPlanner(seed=42)
planner.set_joint_limits(
    min_positions=np.array([-np.pi, -np.pi]),
    max_positions=np.array([np.pi, np.pi]),
)

start = JointState(id=np.arange(2), position=np.zeros(2), velocity=np.zeros(2), torque=np.zeros(2))
goal  = JointState(id=np.arange(2), position=np.array([0.5, 0.5]), velocity=np.zeros(2), torque=np.zeros(2))

planner.plan(start, goal, None)
while not planner.is_planned():
    time.sleep(0.05)

cmd = JointState(id=start.id.copy(), position=np.zeros(2), velocity=np.zeros(2), torque=np.zeros(2))
planner.eval(0.5, cmd)
# cmd.position holds joint angles at 50% along the path
```

---

## Glossary

- **Joint space:** Path and configurations described by joint angles (e.g. rad), not Cartesian end-effector pose.
- **Trajectory:** Sequence of (progress, JointState) from start (0) to goal (1).
- **eval(progress):** Samples the planned path at a given progress (often with smooth time scaling) and writes the result into a `JointState` in-place.
