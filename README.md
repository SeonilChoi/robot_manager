# Robot Manager

Robot control and scheduling library. Loads robot config from YAML, supports multiple robot types and scheduler/planner backends.

## Install

```bash
pip install -e .
```

## Usage

```python
from robot_manager import RobotManager

mgr = RobotManager("config.yaml")
mgr.initialize()
mgr.control()
```

## Config

YAML config must include a `robot` section with `id`, `number_of_joints`, `scheduler_type`, `planner_type`, and `type` (e.g. `little_reader`).

## Test

```bash
python3 -m unittest discover -s tests -v
```
