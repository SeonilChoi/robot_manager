# Robot Manager

Robot control and scheduling library. Loads robot configuration from YAML, supports multiple robot types, and provides FSM scheduler and RRT planner in arbitrary configuration spaces (joint, pose position, velocity, etc.).

## Install

```bash
pip install -e .
```

## Usage

```python
from robot_manager import RobotManager

manager = RobotManager("config/robot_config.yaml")
manager.initialize()

# Control loop: get command, then update with current status
cmd = manager.control()
manager.update(status=current_joint_state, obstacle=obstacle_state)
```

## Config

YAML config must include a `robot` section with:

- `id`: robot id
- `number_of_joints`: number of joints
- `scheduler_type`: e.g. `fsm`
- `planner_type`: e.g. `rrt`
- `type`: robot model, e.g. `little_reader`
- `controller_indexes`: optional list

## Planner (RRT)

RRT runs in any Euclidean config space (e.g. `joint_state.position`, `pose.position`, `velocity.linear`). Use `RrtPlanner` with `generate_trajectory(start_config, goal_config, obstacle_state)` and `eval_config(progress)` for generic spaces, or pass `JointState` for joint space.

## Tests

```bash
python3 -m pytest tests/ -v
```

- **test_scheduler.py**: FSM scheduler transitions and tick.
- **test_planner.py**: RrtAlgorithm, RrtPlanner, and visualization for each config space (outputs in `tests/visualizations/`).
- **test_gui.py**: Config existence and RobotManager init.

## GUI

```bash
python tests/gui_robot_manager.py
```

Requires config at `config/robot_config.yaml`. Buttons: Home, Move, Stop, Auto. Control/update run on timers.
