# scheduler

Scheduler implementations (subclass `core.scheduler.Scheduler`).

- **fsm_scheduler** – `State` (STOPPED, OPERATING, HOMING, INVALID), `Action` (STOP, MOVE, HOME). `TRANSITION_TABLE`, `get_next_state(current, action)`. `FsmScheduler(dt)`: `reset()`, `step()`, `tick(FsmAction)`; HOMING at progress 1.0 transitions to STOPPED.
