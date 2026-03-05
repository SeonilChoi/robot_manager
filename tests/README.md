# Tests

Run from project root:

```bash
python3 -m pytest tests/ -v
```

---

## Layout

| File | Description |
|------|-------------|
| **test_scheduler.py** | FSM scheduler: `get_next_state()` for all state/action pairs; `FsmScheduler` reset, step, tick; HOME until progress 1.0 → STOPPED. |
| **test_planner.py** | RrtAlgorithm (same/near goal, size mismatch); RrtPlanner (eval, plan); **visualization per config space** (joint, pose.position, velocity.linear). Outputs under **tests/visualizations/**: `config_space_joint.png`, `config_space_pose_position.png`, `config_space_velocity_linear.png`. |
| **test_gui.py** | Config file exists; RobotManager initializes with that config (no window). |

---

## Visualizations

Plots are written to `tests/visualizations/`. The directory is created automatically when planner visualization tests run (requires matplotlib).
