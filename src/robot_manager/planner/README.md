# Planner

This package helps the robot **plan a path** in joint space: from where it is now (current joint angles) to where you want it to go (target joint angles), while avoiding obstacles.

---

## Two main parts

### 1. Base planner (threading) — `core.planner.Planner`

The base class runs planning **in a background thread**. That way, when you call `plan()`, your program doesn’t block: the planner keeps working in the background while you do other things.

- You call **`plan(current_state, target_state, obstacle_state)`** to request a new path.
- A worker thread runs **`_run()`**, which calls **`generate_trajectory()`** to compute the path.
- When planning finishes, **`is_planned()`** becomes `True` and you can use **`eval(progress)`** to get joint positions along the path (e.g. `progress=0` = start, `progress=1` = goal).

If you want a different planning method (not RRT), you subclass `Planner` and implement `generate_trajectory()` and `eval()`.

### 2. RRT algorithm — `utils.rrt`

RRT (Rapidly-exploring Random Tree) is the algorithm that actually **finds a path** in joint space and checks for collisions.

- It lives in **`utils.rrt`** (no threading there—just the math).
- **`RrtAlgorithm`** has:
  - **`run(start, goal, obstacle_state)`** → returns `(success, trajectory)`.
  - Optional **`set_joint_limits()`** and **`set_collision_checker()`** to respect joint bounds and obstacles.

You can use `RrtAlgorithm` by itself (e.g. in a script or test) if you don’t need the background thread.

---

## RrtPlanner (what you usually use)

**`RrtPlanner`** in this package ties the two together:

- It **subclasses `Planner`** (so it has `plan()`, `eval()`, `is_planned()`, and the worker thread).
- Inside, it uses **`RrtAlgorithm`** from `utils.rrt`: when the thread runs `generate_trajectory()`, it just calls `_rrt.run()` and stores the result.
- You can call **`set_joint_limits()`** and **`set_collision_checker()`** on `RrtPlanner`; they are passed through to the internal `RrtAlgorithm`.

So: **threading and API come from `Planner`; the path-finding logic comes from `utils.rrt.RrtAlgorithm`.**

---

## Quick example

```python
import time
import numpy as np
from robot_manager.planner import RrtPlanner
from robot_manager.core.planner import JointState

# Create planner (optional: seed for reproducible results)
planner = RrtPlanner(seed=42)

# Optional: set joint limits and collision checker
planner.set_joint_limits(
    min_positions=np.array([-3.14, -3.14]),
    max_positions=np.array([3.14, 3.14]),
)

# Start and goal as joint positions
start = JointState(
    id=np.array([0.0, 1.0]),
    position=np.array([0.0, 0.0]),
    velocity=np.zeros(2),
    torque=np.zeros(2),
)
goal = JointState(
    id=np.array([0.0, 1.0]),
    position=np.array([0.5, 0.5]),
    velocity=np.zeros(2),
    torque=np.zeros(2),
)

# Request a path (returns immediately; planning runs in background)
planner.plan(start, goal, obstacle_state=None)

# Wait until planning is done (e.g. poll in a loop)
while not planner.is_planned():
    time.sleep(0.05)

# Get joint command at 50% along the path
joint_command = JointState(id=start.id.copy(), position=np.zeros(2), velocity=np.zeros(2), torque=np.zeros(2))
planner.eval(0.5, joint_command)
# joint_command.position now has the interpolated joint angles
```

---

## Where everything lives

| What | Where |
|------|--------|
| Base planner (threading, `plan`, `eval`, `_run`) | `robot_manager.core.planner.Planner` |
| Joint state / obstacle types | `robot_manager.core.planner` (`JointState`, `Obstacle`) |
| RRT algorithm (no threading) | `robot_manager.utils.rrt` (`RrtAlgorithm`, `run`, helpers) |
| RRT + planner together | `robot_manager.planner.RrtPlanner` |

---

## Words to know

- **Joint space:** A path described by joint angles (e.g. shoulder, elbow), not by Cartesian positions. The planner works in joint space.
- **Trajectory:** A sequence of (time, joint state) from start (progress 0) to goal (progress 1).
- **eval(progress):** Returns the joint position (and velocity/torque) at a given progress along the planned path (0 = start, 1 = goal). Uses smooth time scaling so motion starts and ends gently.
