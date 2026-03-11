# Planner

Plan path **current → target** in a **background thread**; sample with `eval(progress)`.

```mermaid
flowchart LR
  plan[plan] --> worker[worker thread]
  worker --> G[_generate_trajectory]
  G --> traj[(trajectory)]
  traj --> eval[eval]
  eval --> config[config]
```

**RrtPlanner:** `plan(current, target, obstacles)` → worker runs RRT; `eval(progress)` → config or None (quintic + interpolate). Set `set_bounds()`, `set_collision_checker()` before planning.
