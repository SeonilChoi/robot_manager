# scheduler

Scheduler implementations.

- **fsm_scheduler** – FSM with `State` (STOPPED, OPERATING, HOMING, INVALID), `Action` (STOP, MOVE, HOME), and a transition table. Use `get_next_state(current, action)` for the next state.
