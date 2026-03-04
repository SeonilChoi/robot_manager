# core

Shared types and abstract interfaces.

- **robot** – `Robot` (ABC), `RobotConfig`, `JointState`, `SchedulerType`, `PlannerType`, `to_scheduler_type`, `to_planner_type`.
- **scheduler** – `Scheduler` (ABC): `reset()`, `step()`, `tick(action)`, `_progress_raw(t)`.
- **planner** – `Planner` (ABC): `plan()`.
