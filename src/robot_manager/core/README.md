# core

Abstract interfaces: **Robot**, **Scheduler**, **Planner**; shared types.

```mermaid
flowchart LR
  A[RobotConfig] --> Robot
  Robot --> control
  control --> JointState
  Scheduler --> tick
  tick --> FsmState
  Planner --> plan
  plan --> eval
  eval --> JointState
```

**Robot (ABC):** `initialize()`, `control()`, `update()`.  
**Scheduler (ABC):** `reset()`, `step()`, `tick(action)` → `(changed, FsmState)`.  
**Planner (ABC):** `plan()`, `eval(progress)`, `is_planned()`, `_generate_trajectory()`.
