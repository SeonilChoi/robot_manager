# tests

Unittest suite. Run from project root:

```bash
python3 -m unittest discover -s tests -v
```

- **fsm_scheduler_test** – `get_next_state()` for all state/action pairs; `FsmScheduler` reset, step, tick transitions, HOME until progress 1.0 → STOPPED.
