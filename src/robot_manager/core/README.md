# core

Shared types and abstract interfaces.

- **robot** – `Robot` (ABC), `RobotConfig`, `JointState`, `SchedulerType`, `PlannerType`, `toSchedulerType`, `toPlannerType`.
- **scheduler** – `Scheduler` (ABC): `reset()`, `step()`, `tick(action)`, `_progress_raw(t)`.
- **planner** – `Planner` (ABC): `plan()`.
