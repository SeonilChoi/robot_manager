# robots

Concrete robots: subclass **core.Robot**; implement `initialize`, `control`, `update`, and `_home` / `_move` / `_stop` / `_auto`.

```mermaid
flowchart TB
  Robot[Robot ABC] --> LittleReader
  LittleReader --> FSM[FsmScheduler]
  LittleReader --> RRT[RrtPlanner]
  LittleReader --> FK[get_joint_coordinates]
```

```mermaid
flowchart LR
  status --> update
  obstacles --> update
  update --> coords
  control --> planner
  planner --> cmd
  _home --> plan
  plan --> traj
```

**LittleReader:** Dual-arm; FSM + RRT; DH FK. Collision: **SelfObstacleState** (links), **CircleObstacleState**.  
**Key:** `update` → FK + obstacles; `control` → mode dispatch, `planner.eval(progress)` → command; `_home` → plan to home config.

![LittleReader Home](little_reader_home.png)
